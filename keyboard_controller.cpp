#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <SDL2/SDL.h>
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

class SdlGuard {
public:
  SdlGuard() { ok_ = (SDL_Init(SDL_INIT_VIDEO) == 0); }
  ~SdlGuard() { if (ok_) SDL_Quit(); }
  bool ok() const { return ok_; }
private:
  bool ok_{false};
};

class KeyboardController : public rclcpp::Node {
public:
  KeyboardController()
  : Node("keyboard_controller"),
    lin_speed_(declare_parameter("lin_speed", 0.9)),
    ang_speed_(declare_parameter("ang_speed", 1.6)),
    manual_(true)
  {
    // SDL init
    if (!sdl_.ok()) {
      throw std::runtime_error("SDL initialization failed");
    }
    // Küçük bir pencere; klavye olayları için focus gerekli
    window_ = SDL_CreateWindow(
      "Teleop (W/A/S/D, X: mode, SPACE: stop)",
      SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
      480, 120, SDL_WINDOW_SHOWN
    );
    if (!window_) {
      throw std::runtime_error("SDL_CreateWindow failed");
    }

    pub_ = create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    timer_ = create_wall_timer(16ms, std::bind(&KeyboardController::onTimer, this)); // ~60Hz

    RCLCPP_INFO(get_logger(),
      "Hazır. Mod: MANUEL. Basılı tutma aktif.\n"
      "W/S: ileri/geri, A/D: sol/sağ dönüs, SPACE: durdur, X: mod degistir.");
  }

  ~KeyboardController() override {
    if (window_) SDL_DestroyWindow(window_);
  }

private:
  void onTimer() {
    // 1) Olayları çek (key down/up’ları yakalayıp tek-seferlik eylemler yapacağız)
    SDL_Event e;
    while (SDL_PollEvent(&e)) {
      if (e.type == SDL_QUIT) {
        rclcpp::shutdown();
        return;
      } else if (e.type == SDL_KEYDOWN && !e.key.repeat) {
        if (e.key.keysym.scancode == SDL_SCANCODE_X) {
          manual_ = !manual_;
          RCLCPP_INFO(get_logger(), "Mod: %s", manual_ ? "MANUEL" : "OTOMATIK (sabit daire)");
        } else if (e.key.keysym.scancode == SDL_SCANCODE_SPACE) {
          // acil durdur: anında 0 yayınla
          geometry_msgs::msg::Twist stop;
          pub_->publish(stop);
          return;
        }
      }
    }

    geometry_msgs::msg::Twist twist;

    if (manual_) {
      // 2) Sürekli tuş durumlarını oku (eşzamanlı kombolar için)
      const Uint8* ks = SDL_GetKeyboardState(nullptr);

      const bool W = ks[SDL_SCANCODE_W];
      const bool S = ks[SDL_SCANCODE_S];
      const bool A = ks[SDL_SCANCODE_A];
      const bool D = ks[SDL_SCANCODE_D];

      // Lineer hız: W (+) / S (-)
      double v = 0.0;
      if (W && !S) v =  lin_speed_;
      else if (S && !W) v = -lin_speed_;

      // Açısal hız: A (+) / D (-)  (turtlesim’de sağ dönüş genelde -z)
      double w = 0.0;
      if (A && !D) w =  ang_speed_;
      else if (D && !A) w = -ang_speed_;

      twist.linear.x  = v;
      twist.angular.z = w;
      // Hiçbir tuş yoksa zaten v=0, w=0 → duruş
    } else {
      // 3) Otomatik mod: sabit yarıçaplı daire
      twist.linear.x  = lin_speed_ * 0.8;
      twist.angular.z = ang_speed_ * 0.8;
    }

    pub_->publish(twist);
  }

  // ROS
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double lin_speed_;
  double ang_speed_;
  bool manual_;

  // SDL
  SdlGuard sdl_;
  SDL_Window* window_{nullptr};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<KeyboardController>();
    rclcpp::spin(node);
  } catch (const std::exception& ex) {
    fprintf(stderr, "Fatal: %s\n", ex.what());
  }
  rclcpp::shutdown();
  return 0;
}
