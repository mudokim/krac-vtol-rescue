#pragma once

// Real-time pygame-style BT viewer, ported from py_bt_ros's
// modules/bt_visualiser.py (grid layout by depth, color-coded by NodeStatus,
// pan/zoom via mouse+keyboard). BT.CPP v3 tree nodes don't expose a uniform
// children() accessor across ControlNode/DecoratorNode, so children() below
// dynamic_casts to bridge that.

#include <behaviortree_cpp_v3/bt_factory.h>

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace krac_control::bt {

class BTViewer {
public:
  explicit BTViewer(const std::string& direction = "Vertical");
  ~BTViewer();

  BTViewer(const BTViewer&) = delete;
  BTViewer& operator=(const BTViewer&) = delete;

  // Returns false once the user closes the window or presses ESC/Q.
  bool pollEvents();
  void render(BT::Tree& tree);

private:
  std::vector<BT::TreeNode*> children(BT::TreeNode* node) const;
  void layoutTree(BT::TreeNode* node, int depth, std::map<int, int>& level_tracker,
                   std::map<BT::TreeNode*, std::pair<int, int>>& grid);
  SDL_Color colorFor(BT::NodeStatus status) const;
  std::pair<int, int> gridToPx(int gx, int gy) const;
  std::pair<int, int> transform(int x, int y) const;
  void drawEdge(int px, int py, int cx, int cy, int w, int h, bool active);
  void drawNode(BT::TreeNode* node, int px, int py, int w, int h);
  void drawFilledEllipse(int cx, int cy, int rx, int ry, SDL_Color color);

  std::string direction_;
  SDL_Window* window_ = nullptr;
  SDL_Renderer* renderer_ = nullptr;
  TTF_Font* font_ = nullptr;
  TTF_Font* font_small_ = nullptr;

  float scale_ = 1.0f;
  int pan_x_ = 0;
  int pan_y_ = 0;
  bool dragging_ = false;
  int drag_origin_x_ = 0;
  int drag_origin_y_ = 0;
  int pan_origin_x_ = 0;
  int pan_origin_y_ = 0;

  // While true, render() recomputes scale_/pan_ every frame so the whole
  // tree fills the current window (text stays as large as possible without
  // needing to pan). Any manual zoom/drag turns this off; 'r' turns it back on.
  bool auto_fit_ = true;
  void applyAutoFit(const std::map<BT::TreeNode*, std::pair<int, int>>& positions, int node_w, int node_h);
};

}  // namespace krac_control::bt
