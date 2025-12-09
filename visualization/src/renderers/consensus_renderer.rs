//! Consensus/SwarmRaft visualization rendering

use egui::{Painter, Pos2, Vec2, Color32, Stroke};
use crate::state::{ConsensusVisualState, NodeState, MessageType};

/// Draw Consensus/SwarmRaft visualization
pub fn draw<F>(painter: &Painter, state: &ConsensusVisualState, world_to_screen: F, _scale: f32)
where
    F: Fn(Pos2) -> Pos2,
{
    let leader_pos = state.leader_id.and_then(|id| {
        state.nodes.get(id).map(|n| world_to_screen(n.position))
    });

    // Draw connections from followers to leader
    if let Some(l_pos) = leader_pos {
        for node in &state.nodes {
            if Some(node.id) != state.leader_id {
                let pos = world_to_screen(node.position);

                // Connection line with vote status color
                let color = if node.vote_granted {
                    Color32::from_rgba_premultiplied(100, 255, 100, 100) // Green - voted
                } else {
                    Color32::from_rgba_premultiplied(100, 100, 100, 50) // Gray - no vote
                };
                painter.line_segment([pos, l_pos], Stroke::new(2.0, color));
            }
        }
    }

    // Draw messages in flight
    for msg in &state.messages {
        if let (Some(from_node), Some(to_node)) = (state.nodes.get(msg.from), state.nodes.get(msg.to)) {
            let from_pos = world_to_screen(from_node.position);
            let to_pos = world_to_screen(to_node.position);
            let msg_pos = Pos2::new(
                from_pos.x + (to_pos.x - from_pos.x) * msg.progress,
                from_pos.y + (to_pos.y - from_pos.y) * msg.progress,
            );

            let msg_color = match msg.msg_type {
                MessageType::RequestVote => Color32::from_rgb(255, 200, 50),
                MessageType::VoteGranted => Color32::from_rgb(100, 255, 100),
                MessageType::AppendEntries => Color32::from_rgb(100, 150, 255),
                MessageType::Heartbeat => Color32::from_rgb(255, 100, 100),
            };
            painter.circle_filled(msg_pos, 4.0, msg_color);
        }
    }

    // Draw heartbeat pulses from leader
    if let Some(l_pos) = leader_pos {
        let pulse_progress = (state.term as f32 * 0.2) % 1.0;
        let pulse_radius = 20.0 + pulse_progress * 80.0;
        let pulse_alpha = ((1.0 - pulse_progress) * 60.0) as u8;
        painter.circle_stroke(
            l_pos,
            pulse_radius,
            Stroke::new(2.0, Color32::from_rgba_premultiplied(255, 200, 100, pulse_alpha)),
        );
    }

    // Draw nodes
    for node in &state.nodes {
        let pos = world_to_screen(node.position);
        let is_leader = Some(node.id) == state.leader_id;
        let is_candidate = node.state == NodeState::Candidate;

        // Determine node appearance based on role
        let (color, size, label) = if is_leader {
            (Color32::from_rgb(255, 215, 0), 18.0, "LEADER") // Gold for leader
        } else if is_candidate {
            (Color32::from_rgb(255, 150, 50), 14.0, "CAND") // Orange for candidate
        } else {
            (Color32::from_rgb(100, 180, 100), 12.0, "FLLWR") // Green for follower
        };

        // Outer glow
        painter.circle_filled(
            pos,
            size + 5.0,
            Color32::from_rgba_premultiplied(color.r(), color.g(), color.b(), 40),
        );

        // Main node circle
        painter.circle_filled(pos, size, color);
        painter.circle_stroke(pos, size, Stroke::new(2.0, Color32::WHITE));

        // Role label
        painter.text(
            pos,
            egui::Align2::CENTER_CENTER,
            label,
            egui::FontId::monospace(7.0),
            Color32::BLACK,
        );

        // Node ID below
        painter.text(
            pos + Vec2::new(0.0, size + 10.0),
            egui::Align2::CENTER_TOP,
            format!("N{}", node.id),
            egui::FontId::monospace(9.0),
            Color32::from_rgb(180, 180, 180),
        );

        // Log index indicator (small bar showing commit progress)
        if !is_leader {
            let bar_width = 20.0;
            let bar_height = 4.0;
            let bar_pos = pos + Vec2::new(-bar_width / 2.0, size + 22.0);
            let commit_ratio = if state.committed_index > 0 {
                (node.log_index as f32 / state.committed_index as f32).min(1.0)
            } else {
                0.0
            };

            painter.rect_filled(
                egui::Rect::from_min_size(bar_pos, Vec2::new(bar_width, bar_height)),
                1.0,
                Color32::from_rgb(40, 40, 40),
            );
            painter.rect_filled(
                egui::Rect::from_min_size(bar_pos, Vec2::new(bar_width * commit_ratio, bar_height)),
                1.0,
                Color32::from_rgb(100, 200, 255),
            );
        }
    }

    // Draw term and commit info at bottom
    let center = world_to_screen(Pos2::ZERO);
    painter.text(
        center + Vec2::new(0.0, 90.0),
        egui::Align2::CENTER_CENTER,
        format!("Term: {} | Commit Index: {} | Entries: {}",
            state.term, state.committed_index, state.log_entries.len()),
        egui::FontId::monospace(10.0),
        Color32::from_rgb(200, 200, 200),
    );
}
