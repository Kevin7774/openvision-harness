// xr1rec — external-camera recorder for XR1 experiment logging.
//
// WHY A RESIDENT DAEMON instead of "spawn a recorder per action":
//   opening a UVC camera on macOS costs 0.6-1.5 s (device open + format
//   negotiation + AVCaptureSession start).  The robot must be recording BEFORE
//   it moves, so paying that per action would either delay every motion by a
//   second or race it.  Here the AVCaptureSession stays running for the whole
//   experiment and only AVCaptureMovieFileOutput is started/stopped, which
//   takes ~30-60 ms.
//
// WHY FILE-BASED CONTROL instead of a socket:
//   the Mac only exposes port 22 (verified with a port scan).  A new listener
//   would need a firewall exception; touching a file over an already-open SSH
//   ControlMaster connection costs ~20 ms and needs no new permission.
//
// TCC: camera access is granted to the *bundle*, and the prompt can only be
//   answered in the GUI session, so this binary must live in XR1Rec.app and be
//   launched with `open` (see install_mac_recorder.sh).  A bare sshd-spawned
//   process is never allowed to open the camera.
//
// Modes:
//   xr1rec devices                 list cameras + TCC authorization status
//   xr1rec daemon --dir D [--device SUBSTR] [--fps N] [--preset high|1080|720]
//
// Control protocol, all inside D:
//   ctl/start   <- write clip name (one line)   ; daemon deletes it and records D/clips/<name>.mov
//   ctl/stop    <- touch                        ; daemon stops, finalises the file
//   ctl/quit    <- touch                        ; daemon exits
//   state.json  -> heartbeat + state ("idle"|"recording") + last clip result
//   events.jsonl-> one line per start/stop/error
//   daemon.log  -> free-form log

import AVFoundation
import Foundation

let VERSION = "1.1"

// ── small helpers ────────────────────────────────────────────────────────────

func epoch() -> Double { Date().timeIntervalSince1970 }

func iso(_ t: Double = epoch()) -> String {
    let f = ISO8601DateFormatter()
    f.formatOptions = [.withInternetDateTime, .withFractionalSeconds]
    f.timeZone = TimeZone(identifier: "UTC")
    return f.string(from: Date(timeIntervalSince1970: t))
}

func say(_ s: String) { print(s); fflush(stdout) }

func fail(_ s: String, _ code: Int32 = 1) -> Never {
    FileHandle.standardError.write(("xr1rec: " + s + "\n").data(using: .utf8)!)
    exit(code)
}

func jsonText(_ obj: [String: Any]) -> String {
    guard let d = try? JSONSerialization.data(withJSONObject: obj,
                                              options: [.prettyPrinted, .sortedKeys]),
          let s = String(data: d, encoding: .utf8) else { return "{}" }
    return s
}

func authStatusName() -> String {
    switch AVCaptureDevice.authorizationStatus(for: .video) {
    case .authorized: return "authorized"
    case .denied: return "denied"
    case .restricted: return "restricted"
    case .notDetermined: return "notDetermined"
    @unknown default: return "unknown"
    }
}

/// Ask for camera access and block until the user (or TCC's cache) answers.
/// Returns false in an sshd context with no GUI session -- that is the whole
/// reason install_mac_recorder.sh launches the app with `open`.
///
/// The wait is long (default 15 min) and heartbeats while it waits: the consent
/// dialog only exists on the Mac's physical screen, so whoever is at the robot
/// may need minutes to walk over and click Allow.  Exiting early would tear the
/// dialog's owner out from under it and force another prompt.
/// `timeout <= 0` waits forever, which is the daemon's setting: dying on a
/// deadline is strictly worse than waiting.  A dead daemon means the click has
/// nowhere to land, so whoever finally walks over to the Mac sees no prompt and
/// the robot side needs a relaunch before it can be authorized at all.
func ensureAuthorized(timeout: Double = 0, beat: ((Double) -> Void)? = nil) -> Bool {
    if AVCaptureDevice.authorizationStatus(for: .video) == .authorized { return true }
    let sem = DispatchSemaphore(value: 0)
    var granted = false
    AVCaptureDevice.requestAccess(for: .video) { ok in granted = ok; sem.signal() }
    let t0 = epoch()
    while timeout <= 0 || epoch() - t0 < timeout {
        if sem.wait(timeout: .now() + 1) == .success { return granted }
        // Poll the status as well as the callback: if the dialog gets dismissed
        // and the toggle is flipped later in System Settings > Privacy, the
        // requestAccess completion never fires, but the status does change.
        switch AVCaptureDevice.authorizationStatus(for: .video) {
        case .authorized: return true
        case .denied, .restricted: return false     // explicit no -- stop waiting
        default: break
        }
        beat?(epoch() - t0)
    }
    return false
}

func cameras() -> [AVCaptureDevice] {
    var types: [AVCaptureDevice.DeviceType] = [.builtInWideAngleCamera]
    if #available(macOS 14.0, *) {
        types.insert(.external, at: 0)
        types.append(.continuityCamera)
    }
    let s = AVCaptureDevice.DiscoverySession(deviceTypes: types,
                                             mediaType: .video,
                                             position: .unspecified)
    return s.devices
}

func isExternal(_ d: AVCaptureDevice) -> Bool {
    if #available(macOS 14.0, *) { return d.deviceType == .external }
    return false
}

func fmtLabel(_ f: AVCaptureDevice.Format) -> String {
    let dim = CMVideoFormatDescriptionGetDimensions(f.formatDescription)
    let fps = f.videoSupportedFrameRateRanges.map { $0.maxFrameRate }.max() ?? 0
    return "\(dim.width)x\(dim.height)@\(Int(fps))"
}

func fmtArea(_ f: AVCaptureDevice.Format) -> Int {
    let d = CMVideoFormatDescriptionGetDimensions(f.formatDescription)
    return Int(d.width) * Int(d.height)
}

/// Highest-resolution format that can actually sustain `fps` (ties broken by fps).
/// `formats.first(where:)` is wrong: the list is not ordered by resolution, so it
/// silently picks something like 640x480 and that overrides sessionPreset.
func bestFormat(_ d: AVCaptureDevice, fps: Int) -> AVCaptureDevice.Format? {
    // TOLERANCE IS NOT COSMETIC.  UVC advertises frame intervals in 100 ns
    // units, so "30 fps" arrives as 1e7/333333 = 30.00003, and a fixed-rate
    // format has min == max == 30.00003.  An exact `minFrameRate <= 30` test is
    // then false for EVERY 30 fps format: this camera offers 1920x1080@30 and
    // the strict comparison reported "best_at_30: none", silently leaving the
    // recording at whatever the preset negotiated.
    let eps = 0.2
    let want = Double(fps)
    let usable = d.formats.filter { f in
        fps <= 0 || f.videoSupportedFrameRateRanges.contains {
            $0.minFrameRate <= want + eps && want <= $0.maxFrameRate + eps
        }
    }
    return usable.max { a, b in
        let (aa, ba) = (fmtArea(a), fmtArea(b))
        if aa != ba { return aa < ba }
        let af = a.videoSupportedFrameRateRanges.map { $0.maxFrameRate }.max() ?? 0
        let bf = b.videoSupportedFrameRateRanges.map { $0.maxFrameRate }.max() ?? 0
        return af < bf
    }
}

func describe(_ d: AVCaptureDevice) -> [String: Any] {
    // `formats.last` was NOT the best format, just the last one enumerated --
    // it reported "1920x1080@5" for a camera that also does 1280x720@30.
    let byArea = d.formats.max { fmtArea($0) < fmtArea($1) }
    return ["name": d.localizedName, "uid": d.uniqueID,
            "model": d.modelID, "external": isExternal(d),
            "connected": d.isConnected, "formats": d.formats.count,
            "top_format": byArea.map(fmtLabel) ?? "",
            "best_at_30": bestFormat(d, fps: 30).map(fmtLabel) ?? "none",
            "all_formats": d.formats.map(fmtLabel)]
}

// ── daemon ───────────────────────────────────────────────────────────────────

final class Daemon: NSObject, AVCaptureFileOutputRecordingDelegate {
    let dir: String, ctl: String, clips: String
    let session = AVCaptureSession()
    let output = AVCaptureMovieFileOutput()
    let device: AVCaptureDevice

    var state = "idle"
    var clip: String?
    var startedAt: Double = 0
    var pendingStop = false
    var lastError: String?
    var last: [String: Any] = [:]
    var tick = 0
    /// What the camera was actually configured to, so the robot side can log the
    /// real resolution/fps of each clip instead of the requested one.
    var activeLabel = ""

    init(dir: String, device: AVCaptureDevice, fps: Int, preset: String) throws {
        self.dir = dir
        self.ctl = dir + "/ctl"
        self.clips = dir + "/clips"
        self.device = device
        super.init()

        for d in [dir, ctl, clips] {
            try FileManager.default.createDirectory(atPath: d,
                                                    withIntermediateDirectories: true)
        }

        session.beginConfiguration()
        switch preset {
        case "1080": session.sessionPreset = .hd1920x1080
        case "720":  session.sessionPreset = .hd1280x720
        default:     session.sessionPreset = .high
        }
        let input = try AVCaptureDeviceInput(device: device)
        guard session.canAddInput(input) else { throw err("cannot add camera input") }
        session.addInput(input)
        guard session.canAddOutput(output) else { throw err("cannot add movie output") }
        session.addOutput(output)
        // Keep partial files playable if the daemon is killed mid-clip.
        output.movieFragmentInterval = CMTime(seconds: 2, preferredTimescale: 1)
        session.commitConfiguration()

        // activeFormat is applied AFTER sessionPreset and silently overrides it,
        // so this choice -- not the preset -- decides what actually gets recorded.
        // Pick the largest frame that sustains `fps`; recording the grasp at
        // 640x480 because it happened to be enumerated first is not acceptable.
        if fps > 0, let f = bestFormat(device, fps: fps) {
            // lockForConfiguration MUST succeed before touching activeFormat --
            // `try?` followed by an unconditional assignment would mutate an
            // unlocked device, which is undefined behaviour.
            do {
                try device.lockForConfiguration()
                device.activeFormat = f
                // Frame duration must land INSIDE the format's advertised range
                // or AVFoundation throws NSInvalidArgumentException (an
                // uncatchable crash from Swift).  1/30 is *just outside* a
                // 30.00003 fps range, so clamp instead of assigning 1/fps.
                let want = Double(fps)
                if let r = f.videoSupportedFrameRateRanges.first(where: {
                    $0.minFrameRate <= want + 0.2 && want <= $0.maxFrameRate + 0.2
                }) {
                    let d = CMTimeClampToRange(
                        CMTime(value: 1, timescale: CMTimeScale(fps)),
                        range: CMTimeRange(start: r.minFrameDuration,
                                           end: r.maxFrameDuration))
                    device.activeVideoMinFrameDuration = d
                    device.activeVideoMaxFrameDuration = d
                }
                device.unlockForConfiguration()
                activeLabel = fmtLabel(f)
            } catch {
                lastError = "cannot lock camera for configuration: " +
                            error.localizedDescription
            }
        } else if fps > 0 {
            lastError = "no format sustains \(fps) fps; leaving preset '\(preset)'"
        }
    }

    func err(_ m: String) -> NSError {
        NSError(domain: "xr1rec", code: 1, userInfo: [NSLocalizedDescriptionKey: m])
    }

    func log(_ m: String) {
        let line = "[\(iso())] \(m)\n"
        if let fh = FileHandle(forWritingAtPath: dir + "/daemon.log") {
            fh.seekToEndOfFile(); fh.write(line.data(using: .utf8)!); try? fh.close()
        } else {
            try? line.write(toFile: dir + "/daemon.log", atomically: true, encoding: .utf8)
        }
    }

    func event(_ obj: [String: Any]) {
        var o = obj; o["t"] = epoch(); o["iso"] = iso()
        guard let d = try? JSONSerialization.data(withJSONObject: o, options: [.sortedKeys]),
              var s = String(data: d, encoding: .utf8) else { return }
        s += "\n"
        let p = dir + "/events.jsonl"
        if let fh = FileHandle(forWritingAtPath: p) {
            fh.seekToEndOfFile(); fh.write(s.data(using: .utf8)!); try? fh.close()
        } else {
            try? s.write(toFile: p, atomically: true, encoding: .utf8)
        }
    }

    func publish() {
        var o: [String: Any] = [
            "version": VERSION, "pid": ProcessInfo.processInfo.processIdentifier,
            "hb": epoch(), "hb_iso": iso(), "state": state,
            "session_running": session.isRunning,
            "device": describe(device), "auth": authStatusName(),
            "active_format": activeLabel,
        ]
        if let c = clip { o["clip"] = c; o["started_at"] = startedAt }
        if let e = lastError { o["error"] = e }
        if !last.isEmpty { o["last"] = last }
        try? jsonText(o).write(toFile: dir + "/state.json", atomically: true, encoding: .utf8)
    }

    func start() {
        session.startRunning()
        log("daemon \(VERSION) up, device=\(device.localizedName) uid=\(device.uniqueID) running=\(session.isRunning)")
        event(["ev": "up", "device": device.localizedName])
        publish()
        Timer.scheduledTimer(withTimeInterval: 0.025, repeats: true) { _ in self.poll() }
        RunLoop.main.run()
    }

    /// Read a control file, delete it, return its contents (trimmed).
    func take(_ name: String) -> String? {
        let p = ctl + "/" + name
        guard FileManager.default.fileExists(atPath: p) else { return nil }
        let body = (try? String(contentsOfFile: p, encoding: .utf8)) ?? ""
        try? FileManager.default.removeItem(atPath: p)
        return body.trimmingCharacters(in: .whitespacesAndNewlines)
    }

    func poll() {
        tick += 1

        if take("quit") != nil {
            log("quit requested")
            if output.isRecording { pendingStop = true; output.stopRecording() }
            event(["ev": "down"])
            state = "down"; publish()
            DispatchQueue.main.asyncAfter(deadline: .now() + 1.5) { exit(0) }
            return
        }

        if let name = take("start") {
            let safe = name.isEmpty ? "clip_\(Int(epoch()))" : name
                .replacingOccurrences(of: "/", with: "_")
                .replacingOccurrences(of: " ", with: "_")
            if output.isRecording {
                lastError = "start '\(safe)' ignored: already recording '\(clip ?? "?")'"
                log(lastError!); event(["ev": "error", "msg": lastError!])
            } else if !session.isRunning {
                lastError = "start '\(safe)' failed: capture session is not running"
                log(lastError!); event(["ev": "error", "msg": lastError!])
            } else {
                let url = URL(fileURLWithPath: clips + "/" + safe + ".mov")
                try? FileManager.default.removeItem(at: url)
                clip = safe; lastError = nil
                output.startRecording(to: url, recordingDelegate: self)
                log("start \(safe)")
            }
            publish()
        }

        if take("stop") != nil {
            if output.isRecording {
                pendingStop = true
                output.stopRecording()          // async; finalised in the delegate
                log("stop \(clip ?? "?")")
            } else {
                lastError = "stop ignored: not recording"
                log(lastError!)
            }
            publish()
        }

        if tick % 20 == 0 { publish() }          // ~2 Hz heartbeat
    }

    // AVCaptureFileOutputRecordingDelegate

    func fileOutput(_ o: AVCaptureFileOutput, didStartRecordingTo url: URL,
                    from connections: [AVCaptureConnection]) {
        state = "recording"; startedAt = epoch()
        event(["ev": "start", "clip": clip ?? "?", "path": url.path])
        publish()                                 // <- robot waits for THIS
    }

    func fileOutput(_ o: AVCaptureFileOutput, didFinishRecordingTo url: URL,
                    from connections: [AVCaptureConnection], error: Error?) {
        let attrs = try? FileManager.default.attributesOfItem(atPath: url.path)
        let bytes = (attrs?[.size] as? Int) ?? 0
        var r: [String: Any] = ["clip": clip ?? "?", "path": url.path,
                                "bytes": bytes,
                                "started_at": startedAt, "stopped_at": epoch(),
                                "duration": epoch() - startedAt]
        if let e = error {
            // AVErrorMaximumFileSizeReached etc. still leave a usable file.
            r["error"] = e.localizedDescription
            lastError = e.localizedDescription
            log("finish \(clip ?? "?") WITH ERROR \(e.localizedDescription)")
        } else {
            log("finish \(clip ?? "?") \(bytes) B")
        }
        last = r; state = "idle"; clip = nil; pendingStop = false
        event(["ev": "stop"].merging(r) { a, _ in a })
        publish()
    }
}

// ── argv ─────────────────────────────────────────────────────────────────────

var argv = Array(CommandLine.arguments.dropFirst())
let mode = argv.first ?? "devices"
if !argv.isEmpty { argv.removeFirst() }

func opt(_ name: String) -> String? {
    guard let i = argv.firstIndex(of: "--" + name), i + 1 < argv.count else { return nil }
    return argv[i + 1]
}

switch mode {
case "devices", "probe":
    let devs = cameras()
    var o: [String: Any] = ["version": VERSION, "auth": authStatusName(),
                            "cameras": devs.map(describe)]
    // Bounded here, unlike the daemon: `devices` is a one-shot query someone is
    // waiting on the output of, so it must not block forever.
    if argv.contains("--request") { o["granted_after_request"] = ensureAuthorized(timeout: 120) }
    say(jsonText(o))

case "daemon":
    guard let dir = opt("dir") else { fail("daemon needs --dir") }
    let want = opt("device")
    let fps = Int(opt("fps") ?? "30") ?? 30
    let preset = opt("preset") ?? "high"

    // Publish BEFORE asking, so the robot can tell "consent dialog is waiting on
    // the Mac's screen" apart from "daemon died", and keep publishing while the
    // dialog is up so the robot can report how long the human has been asked for.
    try? FileManager.default.createDirectory(atPath: dir, withIntermediateDirectories: true)
    let awaiting: (Double) -> Void = { waited in
        let js = jsonText(["state": "awaiting_permission", "hb": epoch(), "hb_iso": iso(),
                           "auth": authStatusName(), "version": VERSION,
                           "waited_s": waited,
                           "hint": "Click Allow in the camera prompt on the Mac's screen",
                           "pid": ProcessInfo.processInfo.processIdentifier])
        try? js.write(toFile: dir + "/state.json", atomically: true, encoding: .utf8)
    }
    awaiting(0)

    // Never silently record a black frame: refuse unless TCC actually allows it.
    if !ensureAuthorized(beat: awaiting) {
        let msg = "camera not authorized (status=\(authStatusName())). " +
                  "Launch XR1Rec.app from the GUI session so the prompt can be answered."
        try? jsonText(["state": "fatal", "error": msg, "hb": epoch(), "hb_iso": iso(),
                       "auth": authStatusName()])
            .write(toFile: dir + "/state.json", atomically: true, encoding: .utf8)
        fail(msg, 3)
    }

    let devs = cameras()
    guard !devs.isEmpty else { fail("no cameras found") }
    let dev: AVCaptureDevice
    if let w = want, !w.isEmpty {
        guard let m = devs.first(where: {
            $0.uniqueID == w
                || $0.localizedName.lowercased().contains(w.lowercased())
        }) else {
            fail("no camera matches '\(w)'; have: " + devs.map { $0.localizedName }.joined(separator: ", "))
        }
        dev = m
    } else {
        // Default to the external camera -- the built-in FaceTime cam points at
        // the operator, not at the robot.
        dev = devs.first(where: isExternal) ?? devs[0]
    }
    do {
        try Daemon(dir: dir, device: dev, fps: fps, preset: preset).start()
    } catch { fail("daemon init failed: \(error.localizedDescription)", 4) }

default:
    fail("usage: xr1rec devices [--request] | xr1rec daemon --dir D [--device S] [--fps N] [--preset high|1080|720]")
}
