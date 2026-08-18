# Domain Docs

How the engineering skills should consume this repo's domain documentation when exploring the codebase.

## Before exploring, read these

Canonical (mattpocock/skills convention) — **neither exists yet**, so skip them silently:

- **`CONTEXT.md`** at the repo root (`zed_ws/CONTEXT.md`) — the glossary.
- **`docs/adr/`** — read ADRs that touch the area you're about to work in.

Where this project's domain knowledge actually lives today, and what each is good for:

- **`STATUS.md`** (repo root) — current-state snapshot: measured subsystem values, the ledger, ranked blockers, recovery commands after teleop. Read this before moving the robot.
- **`PITFALLS.md`** (repo root) — numbered failure modes with the criterion that distinguishes each one. Read the sections touching your area before forming a hypothesis; several entries exist specifically because a plausible hypothesis was wrong.
- **`~/.claude/projects/-home-astrabot/memory/`** — one fact per file, indexed by `MEMORY.md`. Cross-session hardware truths and retracted claims. Note that some entries are explicitly marked as overturned — read the body, not just the index line.
- **`~/AGENTS.md`** — the two machines' identity, SSH, camera inventory, `ROS_DOMAIN_ID`.
- **`~/.claude/skills/xr1-robot/SKILL.md`** — the robot control API and its traps.

These are not a glossary and won't behave like one: they're chronological and partly self-correcting. When they disagree, the more recent measurement wins, and `PITFALLS.md` outranks a bare assertion.

If any file above doesn't exist, **proceed silently**. Don't flag its absence; don't suggest creating it upfront. The `/domain-modeling` skill (reached via `/grill-with-docs` and `/improve-codebase-architecture`) creates `CONTEXT.md` and ADRs lazily when terms or decisions actually get resolved.

## File structure

Single-context repo:

```
zed_ws/
├── CLAUDE.md               ← points at docs/agents/*
├── STATUS.md               ← current state (exists)
├── PITFALLS.md             ← failure modes (exists)
├── CONTEXT.md              ← glossary (not yet)
├── docs/
│   ├── agents/             ← these config files
│   └── adr/                ← decisions (not yet)
├── .scratch/               ← issues + specs (see issue-tracker.md)
├── scripts/
└── experiments/
```

## Use the glossary's vocabulary

When your output names a domain concept (in an issue title, a refactor proposal, a hypothesis, a test name), use the term as defined in `CONTEXT.md`. Until that file exists, follow the terms `STATUS.md` and `PITFALLS.md` already use — they carry a settled vocabulary for the hardware (`tcp_link`, `head_pitch`, `真值锚点`, `可达窗口`) and inventing a synonym for one of those makes two notes look like two subjects.

If the concept you need isn't documented anywhere yet, that's a signal — either you're inventing language the project doesn't use (reconsider) or there's a real gap (note it for `/domain-modeling`).

## Flag ADR conflicts

If your output contradicts an existing ADR — or a measured value in `STATUS.md`, or a numbered entry in `PITFALLS.md` — surface it explicitly rather than silently overriding:

> _Contradicts PITFALLS §33 (head_yaw must stay at 0) — but worth reopening because…_
