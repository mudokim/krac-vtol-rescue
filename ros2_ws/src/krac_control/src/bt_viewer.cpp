#include "krac_control/bt/bt_viewer.hpp"

#include <behaviortree_cpp_v3/control_node.h>
#include <behaviortree_cpp_v3/decorator_node.h>

#include <algorithm>
#include <cmath>

namespace krac_control::bt {

namespace {
constexpr int kNodeWidth = 120;
constexpr int kNodeHeight = 30;
constexpr int kXSpacing = 30;
constexpr int kYSpacing = 50;
constexpr int kWindowWidth = 1500;
constexpr int kWindowHeight = 1000;

constexpr SDL_Color kColorRunning{0, 102, 204, 255};
constexpr SDL_Color kColorSuccess{0, 153, 0, 255};
constexpr SDL_Color kColorFailure{204, 0, 0, 255};
constexpr SDL_Color kColorUnknown{128, 128, 128, 255};
constexpr SDL_Color kColorEdgeActive{0, 102, 204, 255};
constexpr SDL_Color kColorEdgeInactive{0, 0, 0, 255};
constexpr SDL_Color kColorText{255, 255, 255, 255};
constexpr SDL_Color kColorBg{255, 255, 255, 255};
constexpr SDL_Color kColorOutline{0, 0, 0, 255};

const char* kFontCandidates[] = {
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
};
}  // namespace

BTViewer::BTViewer(const std::string& direction) : direction_(direction) {
  SDL_Init(SDL_INIT_VIDEO);
  TTF_Init();

  window_ = SDL_CreateWindow("KRAC BT Viewer", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                             kWindowWidth, kWindowHeight, SDL_WINDOW_RESIZABLE);
  renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_ACCELERATED);
  SDL_SetRenderDrawBlendMode(renderer_, SDL_BLENDMODE_BLEND);
  // Smooth scaling for text textures stretched by drawNode()'s scale_-scaled dst rect.
  SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");

  // Rendered once at a high base resolution; drawNode() scales the destination
  // rect by the current zoom/auto-fit factor instead of drawing at native size,
  // so text actually grows to fill the available node box (see drawNode).
  for (const char* path : kFontCandidates) {
    font_ = TTF_OpenFont(path, 28);
    if (font_) {
      break;
    }
  }
  if (font_) {
    for (const char* path : kFontCandidates) {
      font_small_ = TTF_OpenFont(path, 18);
      if (font_small_) {
        break;
      }
    }
  }
}

BTViewer::~BTViewer() {
  if (font_small_) {
    TTF_CloseFont(font_small_);
  }
  if (font_) {
    TTF_CloseFont(font_);
  }
  if (renderer_) {
    SDL_DestroyRenderer(renderer_);
  }
  if (window_) {
    SDL_DestroyWindow(window_);
  }
  TTF_Quit();
  SDL_Quit();
}

bool BTViewer::pollEvents() {
  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    switch (event.type) {
      case SDL_QUIT:
        return false;
      case SDL_KEYDOWN:
        switch (event.key.keysym.sym) {
          case SDLK_ESCAPE:
          case SDLK_q:
            return false;
          case SDLK_EQUALS:
          case SDLK_PLUS:
          case SDLK_KP_PLUS:
            auto_fit_ = false;
            scale_ = std::min(4.0f, scale_ + 0.1f);
            break;
          case SDLK_MINUS:
          case SDLK_KP_MINUS:
            auto_fit_ = false;
            scale_ = std::max(0.2f, scale_ - 0.1f);
            break;
          case SDLK_r:
            auto_fit_ = true;
            break;
          default:
            break;
        }
        break;
      case SDL_MOUSEBUTTONDOWN:
        if (event.button.button == SDL_BUTTON_LEFT) {
          dragging_ = true;
          drag_origin_x_ = event.button.x;
          drag_origin_y_ = event.button.y;
          pan_origin_x_ = pan_x_;
          pan_origin_y_ = pan_y_;
        } else if (event.button.button == SDL_BUTTON_X1 /*unused, keep wheel below*/) {
        }
        break;
      case SDL_MOUSEBUTTONUP:
        if (event.button.button == SDL_BUTTON_LEFT) {
          dragging_ = false;
        }
        break;
      case SDL_MOUSEWHEEL:
        auto_fit_ = false;
        if (event.wheel.y > 0) {
          scale_ = std::min(4.0f, scale_ + 0.1f);
        } else if (event.wheel.y < 0) {
          scale_ = std::max(0.2f, scale_ - 0.1f);
        }
        break;
      case SDL_MOUSEMOTION:
        if (dragging_) {
          auto_fit_ = false;
          pan_x_ = pan_origin_x_ + (event.motion.x - drag_origin_x_);
          pan_y_ = pan_origin_y_ + (event.motion.y - drag_origin_y_);
        }
        break;
      default:
        break;
    }
  }
  return true;
}

std::vector<BT::TreeNode*> BTViewer::children(BT::TreeNode* node) const {
  if (auto* control = dynamic_cast<BT::ControlNode*>(node)) {
    return control->children();
  }
  if (auto* decorator = dynamic_cast<BT::DecoratorNode*>(node)) {
    if (decorator->child() != nullptr) {
      return {decorator->child()};
    }
  }
  return {};
}

void BTViewer::layoutTree(BT::TreeNode* node, int depth, std::map<int, int>& level_tracker,
                          std::map<BT::TreeNode*, std::pair<int, int>>& grid) {
  int axis_pos;
  auto it = level_tracker.find(depth);
  if (it == level_tracker.end()) {
    level_tracker[depth] = 0;
    axis_pos = 0;
  } else {
    axis_pos = ++(it->second);
  }

  if (direction_ == "Horizontal") {
    grid[node] = {depth, axis_pos};
  } else {
    grid[node] = {axis_pos, depth};
  }

  for (BT::TreeNode* child : children(node)) {
    layoutTree(child, depth + 1, level_tracker, grid);
  }
}

SDL_Color BTViewer::colorFor(BT::NodeStatus status) const {
  switch (status) {
    case BT::NodeStatus::RUNNING:
      return kColorRunning;
    case BT::NodeStatus::SUCCESS:
      return kColorSuccess;
    case BT::NodeStatus::FAILURE:
      return kColorFailure;
    case BT::NodeStatus::IDLE:
    default:
      return kColorUnknown;
  }
}

std::pair<int, int> BTViewer::gridToPx(int gx, int gy) const {
  int x = gx * (kNodeWidth + kXSpacing) + kXSpacing;
  int y = gy * (kNodeHeight + kYSpacing) + kYSpacing;
  return {x, y};
}

std::pair<int, int> BTViewer::transform(int x, int y) const {
  return {static_cast<int>(x * scale_) + pan_x_, static_cast<int>(y * scale_) + pan_y_};
}

void BTViewer::drawFilledEllipse(int cx, int cy, int rx, int ry, SDL_Color color) {
  SDL_SetRenderDrawColor(renderer_, color.r, color.g, color.b, color.a);
  rx = std::max(rx, 1);
  ry = std::max(ry, 1);
  for (int dy = -ry; dy <= ry; ++dy) {
    double frac = 1.0 - (static_cast<double>(dy) * dy) / (static_cast<double>(ry) * ry);
    int dx = static_cast<int>(rx * std::sqrt(std::max(0.0, frac)));
    SDL_RenderDrawLine(renderer_, cx - dx, cy + dy, cx + dx, cy + dy);
  }
}

void BTViewer::drawEdge(int px, int py, int cx, int cy, int w, int h, bool active) {
  SDL_Color color = active ? kColorEdgeActive : kColorEdgeInactive;
  SDL_SetRenderDrawColor(renderer_, color.r, color.g, color.b, color.a);

  int sx, sy, ex, ey;
  if (direction_ == "Horizontal") {
    sx = px + w;
    sy = py + h / 2;
    ex = cx;
    ey = cy + h / 2;
  } else {
    sx = px + w / 2;
    sy = py + h;
    ex = cx + w / 2;
    ey = cy;
  }

  if (active) {
    // Fake line thickness (SDL2 has no native stroke width) by drawing a
    // few parallel offset lines, matching py_bt_ros's thicker "active" edge.
    for (int offset = -1; offset <= 1; ++offset) {
      SDL_RenderDrawLine(renderer_, sx + offset, sy, ex + offset, ey);
      SDL_RenderDrawLine(renderer_, sx, sy + offset, ex, ey + offset);
    }
  } else {
    SDL_RenderDrawLine(renderer_, sx, sy, ex, ey);
  }
}

void BTViewer::drawNode(BT::TreeNode* node, int px, int py, int w, int h) {
  SDL_Color fill = colorFor(node->status());
  const bool is_condition = node->type() == BT::NodeType::CONDITION;

  if (is_condition) {
    drawFilledEllipse(px + w / 2, py + h / 2, w / 2, h / 2, fill);
    SDL_SetRenderDrawColor(renderer_, kColorOutline.r, kColorOutline.g, kColorOutline.b, 255);
    // Outline approximated with a second, unfilled ellipse ring.
    for (int dy = -h / 2; dy <= h / 2; ++dy) {
      double frac = 1.0 - (static_cast<double>(dy) * dy) /
                              (static_cast<double>(h / 2) * (h / 2));
      int dx = static_cast<int>((w / 2) * std::sqrt(std::max(0.0, frac)));
      SDL_RenderDrawPoint(renderer_, px + w / 2 - dx, py + h / 2 + dy);
      SDL_RenderDrawPoint(renderer_, px + w / 2 + dx, py + h / 2 + dy);
    }
  } else {
    SDL_Rect rect{px, py, w, h};
    SDL_SetRenderDrawColor(renderer_, fill.r, fill.g, fill.b, 255);
    SDL_RenderFillRect(renderer_, &rect);
    SDL_SetRenderDrawColor(renderer_, kColorOutline.r, kColorOutline.g, kColorOutline.b, 255);
    SDL_RenderDrawRect(renderer_, &rect);
  }

  if (!font_) {
    return;
  }
  std::string label = node->name();
  if (label.empty()) {
    label = node->registrationName();
  }
  if (label.empty()) {
    return;
  }

  // Text is rendered once at a fixed high-resolution font size, then the
  // destination rect is scaled by `scale_` so it grows/shrinks with the node
  // box instead of staying pinned to a constant pixel size (that mismatch —
  // box scaling but text not — was why text looked "too small" at fit-to-window
  // zoom levels).
  TTF_Font* use_font = font_;
  SDL_Surface* surface = TTF_RenderUTF8_Blended(use_font, label.c_str(), kColorText);
  if (surface && surface->w * scale_ > w - 8 && font_small_) {
    SDL_FreeSurface(surface);
    use_font = font_small_;
    surface = TTF_RenderUTF8_Blended(use_font, label.c_str(), kColorText);
  }
  if (!surface) {
    return;
  }
  SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer_, surface);
  if (texture) {
    int text_w = std::min(static_cast<int>(surface->w * scale_), w - 4);
    int text_h = static_cast<int>(surface->h * scale_);
    SDL_Rect dst{px + (w - text_w) / 2, py + (h - text_h) / 2, text_w, text_h};
    SDL_RenderCopy(renderer_, texture, nullptr, &dst);
    SDL_DestroyTexture(texture);
  }
  SDL_FreeSurface(surface);
}

void BTViewer::applyAutoFit(const std::map<BT::TreeNode*, std::pair<int, int>>& positions,
                            int node_w, int node_h) {
  if (positions.empty() || !window_) {
    return;
  }
  int max_x = 0;
  int max_y = 0;
  for (const auto& [node, pos] : positions) {
    max_x = std::max(max_x, pos.first + node_w);
    max_y = std::max(max_y, pos.second + node_h);
  }
  if (max_x <= 0 || max_y <= 0) {
    return;
  }
  int win_w = kWindowWidth;
  int win_h = kWindowHeight;
  SDL_GetWindowSize(window_, &win_w, &win_h);

  const float fit_x = static_cast<float>(win_w) / static_cast<float>(max_x + kXSpacing);
  const float fit_y = static_cast<float>(win_h) / static_cast<float>(max_y + kYSpacing);
  scale_ = std::clamp(std::min(fit_x, fit_y), 0.2f, 6.0f);
  pan_x_ = 0;
  pan_y_ = 0;
}

void BTViewer::render(BT::Tree& tree) {
  if (!renderer_) {
    return;
  }
  SDL_SetRenderDrawColor(renderer_, kColorBg.r, kColorBg.g, kColorBg.b, 255);
  SDL_RenderClear(renderer_);

  BT::TreeNode* root = tree.rootNode();
  if (!root) {
    SDL_RenderPresent(renderer_);
    return;
  }

  std::map<int, int> level_tracker;
  std::map<BT::TreeNode*, std::pair<int, int>> grid;
  layoutTree(root, 0, level_tracker, grid);

  std::map<BT::TreeNode*, std::pair<int, int>> positions;
  for (const auto& [node, grid_xy] : grid) {
    positions[node] = gridToPx(grid_xy.first, grid_xy.second);
  }

  if (auto_fit_) {
    applyAutoFit(positions, kNodeWidth, kNodeHeight);
  }

  const int w = static_cast<int>(kNodeWidth * scale_);
  const int h = static_cast<int>(kNodeHeight * scale_);

  // Edges first so nodes draw on top of the lines.
  for (const auto& [node, pos] : positions) {
    for (BT::TreeNode* child : children(node)) {
      auto [px, py] = transform(pos.first, pos.second);
      auto [cx, cy] = transform(positions[child].first, positions[child].second);
      const bool active = node->status() == BT::NodeStatus::RUNNING &&
                           child->status() == BT::NodeStatus::RUNNING;
      drawEdge(px, py, cx, cy, w, h, active);
    }
  }

  for (const auto& [node, pos] : positions) {
    auto [px, py] = transform(pos.first, pos.second);
    drawNode(node, px, py, w, h);
  }

  SDL_RenderPresent(renderer_);
}

}  // namespace krac_control::bt
