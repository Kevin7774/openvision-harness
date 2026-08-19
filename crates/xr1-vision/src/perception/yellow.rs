use std::collections::VecDeque;

pub(super) fn component_mask(rgb: &image::RgbImage) -> Vec<bool> {
    let mut selected = vec![false; rgb.width() as usize * rgb.height() as usize];
    for component in components(rgb) {
        for index in component {
            selected[index] = true;
        }
    }
    selected
}

pub(super) fn components(rgb: &image::RgbImage) -> Vec<Vec<usize>> {
    let width = rgb.width() as usize;
    let height = rgb.height() as usize;
    let broad = rgb
        .pixels()
        .map(|pixel| {
            let [red, green, blue] = pixel.0.map(f64::from);
            red.max(green) >= 30.0
                && red.min(green) - blue >= 8.0
                && (red - green).abs() <= 0.50 * red.max(green)
        })
        .collect::<Vec<_>>();
    let mut visited = vec![false; broad.len()];
    let mut selected = Vec::new();
    for start in 0..broad.len() {
        if visited[start] || !broad[start] {
            continue;
        }
        let mut queue = VecDeque::from([start]);
        visited[start] = true;
        let mut component = Vec::new();
        let mut sum_red = 0.0;
        let mut sum_green = 0.0;
        let mut sum_chroma = 0.0;
        while let Some(index) = queue.pop_front() {
            component.push(index);
            let [red, green, blue] = rgb
                .get_pixel((index % width) as u32, (index / width) as u32)
                .0;
            sum_red += f64::from(red);
            sum_green += f64::from(green);
            sum_chroma += f64::from(red.min(green).saturating_sub(blue));
            let x = index % width;
            let y = index / width;
            for neighbor_y in y.saturating_sub(1)..=(y + 1).min(height - 1) {
                for neighbor_x in x.saturating_sub(1)..=(x + 1).min(width - 1) {
                    let neighbor = neighbor_y * width + neighbor_x;
                    if !visited[neighbor] && broad[neighbor] {
                        visited[neighbor] = true;
                        queue.push_back(neighbor);
                    }
                }
            }
        }
        let area = component.len();
        let mean_chroma = sum_chroma / area as f64;
        // Thresholds measured on frames 20260818-112803 and 20260818-170043.
        // The two-sided R/G window rejects both the green cube and orange pads.
        if (20..=5000).contains(&area)
            && sum_red >= 0.85 * sum_green
            && sum_red <= 1.15 * sum_green
            && mean_chroma >= 10.0
        {
            selected.push(component);
        }
    }
    selected
}

#[cfg(test)]
mod tests {
    use super::*;

    fn selected_pixels(rgb: [u8; 3]) -> usize {
        let mut image = image::RgbImage::new(40, 40);
        for pixel in image.pixels_mut() {
            *pixel = image::Rgb(rgb);
        }
        component_mask(&image)
            .into_iter()
            .filter(|kept| *kept)
            .count()
    }

    #[test]
    fn mask_keeps_the_yellow_block_and_drops_the_green_cube() {
        assert!(selected_pixels([179, 184, 81]) > 0);
        assert_eq!(selected_pixels([96, 149, 69]), 0);
    }

    #[test]
    fn orange_pads_are_not_yellow_but_the_block_still_is() {
        let mut image = image::RgbImage::from_pixel(70, 20, image::Rgb([230, 230, 230]));
        for y in 0..20 {
            for x in 0..20 {
                image.put_pixel(x, y, image::Rgb([176, 178, 41]));
            }
            for x in 25..40 {
                image.put_pixel(x, y, image::Rgb([160, 91, 24]));
            }
            for x in 45..60 {
                image.put_pixel(x, y, image::Rgb([161, 122, 91]));
            }
        }
        let mask = component_mask(&image);
        let count = |start: usize, end: usize| {
            (start..end)
                .flat_map(|x| (0..20).map(move |y| y * 70 + x))
                .filter(|index| mask[*index])
                .count()
        };
        assert_eq!(count(0, 20), 400);
        assert_eq!(count(25, 40), 0);
        assert_eq!(count(45, 60), 0);
    }

    #[test]
    fn accepted_components_remain_distinct_for_near_field_ambiguity_checks() {
        let mut image = image::RgbImage::from_pixel(80, 40, image::Rgb([0, 0, 0]));
        for y in 5..15 {
            for x in 5..15 {
                image.put_pixel(x, y, image::Rgb([176, 178, 41]));
            }
            for x in 50..60 {
                image.put_pixel(x, y, image::Rgb([176, 178, 41]));
            }
        }
        let found = components(&image);
        assert_eq!(found.len(), 2);
        assert_eq!(found[0].len(), 100);
        assert_eq!(found[1].len(), 100);
    }
}
