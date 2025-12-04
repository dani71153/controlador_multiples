#include <memory>
#include <queue>
#include <vector>
#include <sstream>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/time.h"

using std::placeholders::_1;
using std::placeholders::_2;

class ControladorMesas : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    const double TOLERANCIA_POS = 0.25;
    const double TOLERANCIA_ANG = 0.20;
    const double TIEMPO_ESTABILIDAD = 1.5;
    const double TIMEOUT_NAV = 50.0;
    const double TIMEOUT_VALIDACION = 8.0;
    const double REINICIO_MOVIMIENTO = 0.15;
    const double REINICIO_YAW = 0.25;
    static constexpr const char* FRAME_MAP = "map";

    enum State { IDLE, NAVIGATING, WAITING_ODOM_CONFIRM, ERROR };
    enum ModoOperacion { MODO_PEDIDO, MODO_ENTREGA, MODO_PAGO, MODO_AUTO };

    ControladorMesas() : Node("controlador_mesas")
    {
        modo_ = MODO_PEDIDO;
        automatico_activo_ = false;
        ultimo_estado_ = "IDLE";
        ultimo_validacion_ = "";
        nav2_activo_ = false;

        odom_recibida_ = false;
        odom_last_time_ = this->now();

        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_buffer_->setUsingDedicatedThread(true);
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        pub_estado_ = this->create_publisher<std_msgs::msg::String>("controlador_mesas/estado", 20);
        pub_cola_actual_ = this->create_publisher<std_msgs::msg::String>("controlador_mesas/cola_actual", 20);
        pub_listado_colas_ = this->create_publisher<std_msgs::msg::String>("controlador_mesas/listado_colas", 20);

        pub_odom_recibida_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("controlador_mesas/odom_recibida", 20);
        pub_odom_mapa_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("controlador_mesas/odom_en_mapa", 20);
        pub_distancia_objetivo_ = this->create_publisher<std_msgs::msg::Float32>("controlador_mesas/distancia_objetivo", 20);

        pub_yaw_odom_ = this->create_publisher<std_msgs::msg::Float32>("controlador_mesas/yaw_odom", 20);
        pub_yaw_mapa_ = this->create_publisher<std_msgs::msg::Float32>("controlador_mesas/yaw_mapa", 20);
        pub_yaw_objetivo_ = this->create_publisher<std_msgs::msg::Float32>("controlador_mesas/yaw_objetivo", 20);
        pub_yaw_diff_ = this->create_publisher<std_msgs::msg::Float32>("controlador_mesas/yaw_diff", 20);

        pub_validacion_ = this->create_publisher<std_msgs::msg::String>("controlador_mesas/validacion_estado", 20);
        pub_objetivos_descartados_ = this->create_publisher<std_msgs::msg::Float32>("controlador_mesas/objetivos_descartados", 20);
        pub_modo_actual_ = this->create_publisher<std_msgs::msg::String>("controlador_mesas/modo_actual", 20);
        pub_en_ejecucion_ = this->create_publisher<std_msgs::msg::Bool>("controlador_mesas/en_ejecucion", 10);
        pub_descartados_lista_ = this->create_publisher<std_msgs::msg::String>("controlador_mesas/objetivos_descartados_lista", 20);
        auto qos_latched = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        pub_verificacion_origen_ = this->create_publisher<std_msgs::msg::Bool>("controlador_mesas/verificacion_del_origen", qos_latched);
        pub_origen_guardado_ = this->create_publisher<geometry_msgs::msg::Pose2D>("controlador_mesas/origen_guardado", qos_latched);

        pub_odom_timeout_ = this->create_publisher<std_msgs::msg::String>("controlador_mesas/odom_timeout", 20);

        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 50, std::bind(&ControladorMesas::callbackOdom, this, _1));

        sub_pedidos_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "entradas_pedidos", 10, std::bind(&ControladorMesas::callbackPedido, this, _1));

        sub_entregas_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "entradas_entregas", 10, std::bind(&ControladorMesas::callbackEntrega, this, _1));

        sub_pagos_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "entradas_pagos", 10, std::bind(&ControladorMesas::callbackPago, this, _1));

        sub_limpiar_ = this->create_subscription<std_msgs::msg::String>(
            "controlador_mesas/limpiar_cola", 10,
            std::bind(&ControladorMesas::callbackLimpiarCola, this, _1));

        sub_timeout_ = this->create_subscription<std_msgs::msg::String>(
            "controlador_mesas/timeout_control", 10,
            std::bind(&ControladorMesas::callbackTimeoutControl, this, _1));

        sub_reiniciar_ = this->create_subscription<std_msgs::msg::String>(
            "controlador_mesas/reiniciar", 10,
            std::bind(&ControladorMesas::callbackReiniciar, this, _1));

        sub_modo_ = this->create_subscription<std_msgs::msg::String>(
            "controlador_mesas/cambiar_modo", 10,
            std::bind(&ControladorMesas::callbackCambiarModo, this, _1));

        sub_autorizar_entrega_ = this->create_subscription<std_msgs::msg::Bool>(
            "controlador_mesas/autorizar_entrega", 10,
            std::bind(&ControladorMesas::callbackAutorizarEntrega, this, _1));

        sub_establecer_origen_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "controlador_mesas/establecer_origen", 10,
            std::bind(&ControladorMesas::callbackEstablecerOrigen, this, _1));

        sub_pagado_ = this->create_subscription<std_msgs::msg::Bool>(
            "controlador_mesas/pagado", 10, std::bind(&ControladorMesas::callbackPagado, this, _1));

        auto nav2_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        sub_nav2_estado_ = this->create_subscription<std_msgs::msg::Bool>(
            "/nav2_lifecycle_manager_navigation/is_active",
            nav2_qos,
            std::bind(&ControladorMesas::callbackNav2Estado, this, _1)
        );

        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        state_ = IDLE;

        verificarNav2ActionServer();

        timer_publicacion_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ControladorMesas::tickPublicacion, this)
        );

        timer_odom_watchdog_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&ControladorMesas::tickOdomWatchdog, this)
        );

        timer_nav2_watchdog_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&ControladorMesas::tickNav2Watchdog, this)
        );

           RCLCPP_INFO(this->get_logger(), "Nodo iniciado con Nav2-check integrado.");
        RCLCPP_INFO(this->get_logger(), "=== INICIALIZACIÓN DEL CONTROLADOR DE MESAS ===");

        RCLCPP_INFO(this->get_logger(), "[INIT] Colas creadas:");
        RCLCPP_INFO(this->get_logger(), "   - Cola pedidos");
        RCLCPP_INFO(this->get_logger(), "   - Cola entregas");
        RCLCPP_INFO(this->get_logger(), "   - Cola pagos");

        RCLCPP_INFO(this->get_logger(), "[INIT] Publishers creados:");
        RCLCPP_INFO(this->get_logger(), "   - controlador_mesas/estado");
        RCLCPP_INFO(this->get_logger(), "   - controlador_mesas/cola_actual");
        RCLCPP_INFO(this->get_logger(), "   - controlador_mesas/listado_colas");
        RCLCPP_INFO(this->get_logger(), "   - controlador_mesas/odom_recibida");
        RCLCPP_INFO(this->get_logger(), "   - controlador_mesas/odom_en_mapa");
        RCLCPP_INFO(this->get_logger(), "   - controlador_mesas/distancia_objetivo");
        RCLCPP_INFO(this->get_logger(), "   - controlador_mesas/yaw_*");
        RCLCPP_INFO(this->get_logger(), "   - controlador_mesas/validacion_estado");
        RCLCPP_INFO(this->get_logger(), "   - controlador_mesas/objetivos_descartados");
        RCLCPP_INFO(this->get_logger(), "   - controlador_mesas/modo_actual");
        RCLCPP_INFO(this->get_logger(), "   - controlador_mesas/objetivos_descartados_lista");
        RCLCPP_INFO(this->get_logger(), "   - controlador_mesas/en_ejecucion");
        RCLCPP_INFO(this->get_logger(), "   - controlador_mesas/odom_timeout");
        RCLCPP_INFO(this->get_logger(), "   - controlador_mesas/verificacion_del_origen");
        RCLCPP_INFO(this->get_logger(), "   - controlador_mesas/origen_guardado");

        RCLCPP_INFO(this->get_logger(), "[INIT] Subscribers creados:");
        RCLCPP_INFO(this->get_logger(), "   - /odom");
        RCLCPP_INFO(this->get_logger(), "   - entradas_pedidos");
        RCLCPP_INFO(this->get_logger(), "   - entradas_entregas");
        RCLCPP_INFO(this->get_logger(), "   - entradas_pagos");
        RCLCPP_INFO(this->get_logger(), "   - controlador_mesas/limpiar_cola");
        RCLCPP_INFO(this->get_logger(), "   - controlador_mesas/timeout_control");
        RCLCPP_INFO(this->get_logger(), "   - controlador_mesas/reiniciar");
        RCLCPP_INFO(this->get_logger(), "   - controlador_mesas/cambiar_modo");
        RCLCPP_INFO(this->get_logger(), "   - controlador_mesas/autorizar_entrega");
        RCLCPP_INFO(this->get_logger(), "   - /nav2_lifecycle_manager_navigation/is_active");
        RCLCPP_INFO(this->get_logger(), "   - controlador_mesas/establecer_origen");
        RCLCPP_INFO(this->get_logger(), "   - pagado");

        RCLCPP_INFO(this->get_logger(), "[INIT] Sistema de TF listo.");
        RCLCPP_INFO(this->get_logger(), "[INIT] Action Client navigate_to_pose inicializado.");

        bool nav2_now = nav2Disponible();
        RCLCPP_INFO(this->get_logger(), "[INIT] Estado Nav2: %s", nav2_now ? "Disponible" : "No disponible");
        publicarModoActual();

        RCLCPP_INFO(this->get_logger(), "[INIT] Estado inicial del nodo: IDLE");
        RCLCPP_INFO(this->get_logger(), "=== CONTROLADOR DE MESAS LISTO ===");

    }

private:
    ModoOperacion modo_;
    bool automatico_activo_;
    State state_;

    bool nav2_activo_;
    bool nav2_msg_recibido_ = false;
    bool nav2_action_ready_prev_ = false;
    bool nav2_action_ready_init_reported_ = false;
    bool nav2_nodos_detectados_ = false;
    bool reinicio_en_progreso_ = false;
    bool entrega_autorizada_ = false; // autorización por lote de entregas
    bool esperando_confirmacion_pago_ = false;

    bool odom_recibida_;
    rclcpp::Time odom_last_time_;
    bool robot_en_origen_ = false;

    geometry_msgs::msg::Pose2D origen_guardado_;
    bool origen_establecido_ = false;
    bool verificacion_origen_publicada_ = false;
    bool ultimo_verificacion_origen_ = false;

    std::queue<geometry_msgs::msg::PoseStamped> cola_pedidos_;
    std::queue<geometry_msgs::msg::PoseStamped> cola_entregas_;
    std::queue<geometry_msgs::msg::PoseStamped> cola_pagos_;
    std::queue<geometry_msgs::msg::PoseStamped>* cola_activa_ = nullptr;
    struct ObjetivoDescartado {
        geometry_msgs::msg::PoseStamped pose;
        std::string motivo;
    };
    std::queue<ObjetivoDescartado> cola_descartados_;

    nav_msgs::msg::Odometry ultima_odom_;
    rclcpp::Time start_nav_time_;
    rclcpp::Time odom_stable_start_;
    rclcpp::Time inicio_validacion_odom_;

    std::string ultimo_estado_;
    std::string ultimo_validacion_;

    bool odom_stable_ = false;
    bool timeout_habilitado_ = false;
    int objetivos_descartados_ = 0;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_estado_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_cola_actual_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_listado_colas_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_odom_recibida_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_odom_mapa_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_distancia_objetivo_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_yaw_odom_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_yaw_mapa_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_yaw_objetivo_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_yaw_diff_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_validacion_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_objetivos_descartados_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_modo_actual_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_en_ejecucion_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_descartados_lista_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_odom_timeout_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_verificacion_origen_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pub_origen_guardado_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pedidos_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_entregas_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pagos_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_limpiar_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_timeout_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_reiniciar_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_modo_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_nav2_estado_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_autorizar_entrega_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr sub_establecer_origen_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_pagado_;

    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    GoalHandleNav::SharedPtr goal_handle_;

    rclcpp::TimerBase::SharedPtr timer_publicacion_;
    rclcpp::TimerBase::SharedPtr timer_odom_watchdog_;
    rclcpp::TimerBase::SharedPtr timer_nav2_watchdog_;

    void verificarNav2ActionServer()
    {
        bool listo = nav_client_->wait_for_action_server(std::chrono::seconds(2));
        nav2_activo_ = listo;
        RCLCPP_INFO(this->get_logger(),
                    "[INIT] Nav2 action server navigate_to_pose: %s",
                    listo ? "ACTIVO" : "INACTIVO");
    }

    bool nav2Disponible()
    {
        bool action_ok = nav_client_->action_server_is_ready() ||
                         nav_client_->wait_for_action_server(std::chrono::seconds(1));

        // Si recibimos lifecycle y dice inactivo, respetamos ese estado; de lo contrario,
        // permitimos navegar si el action server está listo.
        if (nav2_msg_recibido_ && !nav2_activo_) return false;

        return action_ok;
    }

    double yawFromQuat(const geometry_msgs::msg::Quaternion &q) const {
        double siny = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z);
        return std::atan2(siny, cosy);
    }

    double normalizarAngulo(double ang) const {
        return std::atan2(std::sin(ang), std::cos(ang));
    }

    geometry_msgs::msg::Quaternion quatFromYaw(double yaw) const {
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        return tf2::toMsg(q);
    }

    bool transformPose(const geometry_msgs::msg::PoseStamped &pose_in,
                       const std::string &frame_destino,
                       geometry_msgs::msg::PoseStamped &pose_out,
                       const std::string &tag_log)
    {
        try {
            auto tf = tf_buffer_->lookupTransform(
                frame_destino,
                pose_in.header.frame_id,
                tf2::TimePointZero, // usa el último TF disponible para evitar extrapolaciones
                tf2::durationFromSec(0.5));
            tf2::doTransform(pose_in, pose_out, tf);
            pose_out.header.frame_id = frame_destino;
            pose_out.header.stamp = pose_in.header.stamp;
            return true;
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                 "[%s] Transform %s->%s no disponible: %s",
                                 tag_log.c_str(),
                                 pose_in.header.frame_id.c_str(),
                                 frame_destino.c_str(),
                                 ex.what());
        } catch (const std::exception &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                 "[%s] Error transformando %s->%s: %s",
                                 tag_log.c_str(),
                                 pose_in.header.frame_id.c_str(),
                                 frame_destino.c_str(),
                                 ex.what());
        } catch (...) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                 "[%s] Error desconocido en transform %s->%s",
                                 tag_log.c_str(),
                                 pose_in.header.frame_id.c_str(),
                                 frame_destino.c_str());
        }
        return false;
    }

    bool poseEnMapa(const geometry_msgs::msg::PoseStamped &pose_in,
                    geometry_msgs::msg::PoseStamped &pose_out)
    {
        return transformPose(pose_in, FRAME_MAP, pose_out, "ORIGEN");
    }

    void publicarEstado(const std::string &s) {
        ultimo_estado_ = s;
        std_msgs::msg::String m; m.data = s;
        pub_estado_->publish(m);
    }

    void publicarValidacion(const std::string &s) {
        ultimo_validacion_ = s;
        std_msgs::msg::String m; m.data = s;
        pub_validacion_->publish(m);
    }

    void tickPublicacion() {
        publicarCola();
        std_msgs::msg::String est;
        est.data = ultimo_estado_;
        pub_estado_->publish(est);
        std_msgs::msg::String v;
        v.data = ultimo_validacion_;
        pub_validacion_->publish(v);
        std_msgs::msg::Float32 od;
        od.data = objetivos_descartados_;
        pub_objetivos_descartados_->publish(od);
        std_msgs::msg::String modo_msg;
        modo_msg.data = modoActualString();
        pub_modo_actual_->publish(modo_msg);
        std_msgs::msg::String desc_list;
        desc_list.data = listaDescartados();
        pub_descartados_lista_->publish(desc_list);
        std_msgs::msg::Bool ejec;
        ejec.data = true;
        pub_en_ejecucion_->publish(ejec);
        if (origen_establecido_) {
            pub_origen_guardado_->publish(origen_guardado_);
        }
        std_msgs::msg::Bool verif;
        verif.data = robot_en_origen_;
        pub_verificacion_origen_->publish(verif);
    }

    void tickOdomWatchdog() {
        rclcpp::Time now = this->now();
        double diff = (now - odom_last_time_).seconds();

        if (diff > 2.0) {
            std_msgs::msg::String msg;
            msg.data = "odom_timeout";
            pub_odom_timeout_->publish(msg);
            publicarEstado("ERROR: Odometría no recibida.");
            state_ = ERROR;
        }
    }

    void callbackNav2Estado(const std_msgs::msg::Bool::SharedPtr msg)
    {
        nav2_activo_ = msg->data;
        nav2_msg_recibido_ = true;
        publicarEstado(nav2_activo_ ? "Nav2 ACTIVO." : "Nav2 INACTIVO.");
        RCLCPP_INFO(this->get_logger(), "[NAV2] Estado lifecycle is_active: %s", nav2_activo_ ? "true" : "false");
    }

    void tickNav2Watchdog()
    {
        bool listo = nav_client_->action_server_is_ready();

        if (!nav2_action_ready_init_reported_ || listo != nav2_action_ready_prev_) {
            RCLCPP_INFO(this->get_logger(),
                        "[NAV2] Action server navigate_to_pose: %s",
                        listo ? "ACTIVO" : "INACTIVO");
            nav2_action_ready_prev_ = listo;
            nav2_action_ready_init_reported_ = true;
        }

        if (!nav2_msg_recibido_) {
            if (nav2_nodos_detectados_) return;
            auto nav2_nodes = detectarNodosNav2();
            if (!nav2_nodes.empty()) {
                std::stringstream ss;
                ss << "[NAV2] Nodos detectados: ";
                bool first = true;
                for (auto &n : nav2_nodes) {
                    if (!first) ss << ", ";
                    ss << n;
                    first = false;
                }
                RCLCPP_INFO(this->get_logger(), "%s (sin mensajes is_active)", ss.str().c_str());
                nav2_nodos_detectados_ = true;
            } else {
                RCLCPP_WARN(this->get_logger(),
                            "[NAV2] No se detectan nodos Nav2 en el grafo (Nav2 no inicializado).");
            }
        }
    }

    std::vector<std::string> detectarNodosNav2()
    {
        static const std::vector<std::string> patrones = {
            "nav2_lifecycle_manager_navigation",
            "controller_server",
            "planner_server",
            "smoother_server",
            "recoveries_server",
            "bt_navigator"
        };

        std::vector<std::string> encontrados;
        auto node_names = this->get_node_graph_interface()->get_node_names();
        for (const auto &name : node_names) {
            for (const auto &pat : patrones) {
                if (name.find(pat) != std::string::npos) {
                    encontrados.push_back(name);
                    break;
                }
            }
        }
        return encontrados;
    }

    void callbackPedido(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        cola_pedidos_.push(*msg);
        publicarEstado("Pedido añadido.");
        intentarIniciar();
    }

    void callbackEntrega(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        bool estaba_vacia = cola_entregas_.empty();
        cola_entregas_.push(*msg);
        publicarEstado("Entrega añadida.");
        if (modo_ == MODO_ENTREGA && estaba_vacia) {
            entrega_autorizada_ = false; // nueva tanda requiere confirmación
            RCLCPP_WARN(this->get_logger(), "[ENTREGA] Cola de entregas lista, esperando confirmación del operador.");
        }
        intentarIniciar();
    }

    void callbackPago(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        cola_pagos_.push(*msg);
        publicarEstado("Pago añadido.");
        intentarIniciar();
    }

    void callbackPagado(const std_msgs::msg::Bool::SharedPtr msg) {
        if (!esperando_confirmacion_pago_) {
            if (msg->data) {
                RCLCPP_INFO(this->get_logger(),
                            "[PAGO] Confirmación recibida sin estar esperando; se ignora.");
            }
            return;
        }

        if (!msg->data) {
            publicarEstado("Esperando confirmación de pago (pagado=0).");
            return;
        }

        esperando_confirmacion_pago_ = false;
        publicarEstado("Pago confirmado, continuando con la ruta.");
        publicarValidacion("pago_confirmado");
        encolarOrigenSiColaVacia();
        postTarea();
    }

    void callbackTimeoutControl(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "on") timeout_habilitado_ = true;
        else if (msg->data == "off") timeout_habilitado_ = false;
    }

    void callbackCambiarModo(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "pedido") {
            modo_ = MODO_PEDIDO;
            automatico_activo_ = false;
        }
        else if (msg->data == "entrega") {
            modo_ = MODO_ENTREGA;
            automatico_activo_ = false;
        }
        else if (msg->data == "pago") {
            modo_ = MODO_PAGO;
            automatico_activo_ = false;
        }
        else if (msg->data == "auto") {
            modo_ = MODO_PEDIDO;
            automatico_activo_ = true;
        }
        entrega_autorizada_ = false;
        esperando_confirmacion_pago_ = false;
        publicarModoActual();
        intentarIniciar();
    }

    void callbackAutorizarEntrega(const std_msgs::msg::Bool::SharedPtr msg) {
        entrega_autorizada_ = msg->data;
        if (!entrega_autorizada_) {
            RCLCPP_INFO(this->get_logger(), "[ENTREGA] Autorización REVOCADA");
            return;
        }
        if (cola_entregas_.empty()) {
            RCLCPP_WARN(this->get_logger(), "[ENTREGA] Autorización otorgada pero cola de entregas vacía.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "[ENTREGA] Autorización OTORGADA (lote de entregas).");
        intentarIniciar();
    }

    void callbackLimpiarCola(const std_msgs::msg::String::SharedPtr) {
        if (cola_activa_) while (!cola_activa_->empty()) cola_activa_->pop();

        if (state_ == NAVIGATING && goal_handle_) {
            nav_client_->async_cancel_goal(goal_handle_);
        }

        state_ = IDLE;
        if (modo_ == MODO_ENTREGA) entrega_autorizada_ = false;
        esperando_confirmacion_pago_ = false;
    }

    void callbackReiniciar(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "reiniciar") reiniciarNodo();
    }

    void callbackEstablecerOrigen(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
        establecerOrigen(msg->x, msg->y, msg->theta);
        RCLCPP_INFO(this->get_logger(),
                    "[ORIGEN] Origen actualizado manualmente a (%.3f, %.3f, %.3f rad)",
                    msg->x, msg->y, msg->theta);
    }

    void reiniciarNodo() {
        if (reinicio_en_progreso_) {
            RCLCPP_WARN(this->get_logger(), "[REINICIO] Ya en progreso, se ignora solicitud.");
            return;
        }
        reinicio_en_progreso_ = true;
        RCLCPP_WARN(this->get_logger(), "[REINICIO] Reinicio solicitado (inicio).");

        // Cancelar cualquier objetivo activo o pendiente, aun si goal_handle_ no está disponible.
        try {
            nav_client_->async_cancel_all_goals();
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "[REINICIO] Error cancelando metas: %s", e.what());
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "[REINICIO] Error desconocido cancelando metas.");
        }

        if (goal_handle_) {
            try {
                auto st = goal_handle_->get_status();
                if (st == rclcpp_action::GoalStatus::STATUS_ACCEPTED ||
                    st == rclcpp_action::GoalStatus::STATUS_EXECUTING)
                {
                    nav_client_->async_cancel_goal(goal_handle_);
                }
            } catch (...) {}
        }

        while (!cola_pedidos_.empty()) cola_pedidos_.pop();
        while (!cola_entregas_.empty()) cola_entregas_.pop();
        while (!cola_pagos_.empty()) cola_pagos_.pop();
        while (!cola_descartados_.empty()) cola_descartados_.pop();

        timeout_habilitado_ = false;
        odom_stable_ = false;
        objetivos_descartados_ = 0;
        origen_establecido_ = false;
        esperando_confirmacion_pago_ = false;

        goal_handle_.reset();
        state_ = IDLE;
        automatico_activo_ = false;
        modo_ = MODO_PEDIDO;
        entrega_autorizada_ = false;
        ultimo_estado_ = "IDLE";
        ultimo_validacion_.clear();

        publicarModoActual();
        publicarEstado("Reiniciado.");
        publicarValidacion("reiniciado");
        publicarCola();
        reinicio_en_progreso_ = false;
        RCLCPP_INFO(this->get_logger(), "[REINICIO] Reinicio completado.");
    }

    void publicarCola() {
        std_msgs::msg::String actual;
        std_msgs::msg::String lista;

        actualizarColaActiva();
        auto *cola = cola_activa_;

        if (!cola || cola->empty()) {
            actual.data = "Sin mesa actual.";
            lista.data = "Cola completa: []";
            pub_cola_actual_->publish(actual);
            pub_listado_colas_->publish(lista);
            return;
        }

        auto &m = cola->front();
        {
            std::stringstream ss;
            ss << "(" << m.pose.position.x << ", " << m.pose.position.y << ")";
            actual.data = ss.str();
        }

        std::queue<geometry_msgs::msg::PoseStamped> copia = *cola;
        std::stringstream ss;
        ss << "Cola completa: [";
        bool first = true;

        while (!copia.empty()) {
            auto &m2 = copia.front();
            if (!first) ss << ", ";
            ss << "(" << m2.pose.position.x << ", " << m2.pose.position.y << ")";
            first = false;
            copia.pop();
        }

        ss << "]";
        lista.data = ss.str();

        pub_cola_actual_->publish(actual);
        pub_listado_colas_->publish(lista);
    }

    void actualizarColaActiva() {
        if      (modo_ == MODO_PEDIDO)  cola_activa_ = &cola_pedidos_;
        else if (modo_ == MODO_ENTREGA) cola_activa_ = &cola_entregas_;
        else                            cola_activa_ = &cola_pagos_;
    }

    void intentarIniciar() {
        if (state_ != IDLE) return;
        if (esperando_confirmacion_pago_) {
            RCLCPP_INFO(this->get_logger(),
                        "[PAGO] Bloqueado hasta recibir confirmación de pago (topic pagado).");
            return;
        }

        actualizarColaActiva();
        if (!cola_activa_) {
            RCLCPP_DEBUG(this->get_logger(), "[NAV] Sin cola activa definida.");
            return;
        }
        if (cola_activa_->empty()) {
            RCLCPP_DEBUG(this->get_logger(), "[NAV] Cola activa vacía, nada que iniciar.");
            return;
        }

        if (modo_ == MODO_ENTREGA && !entrega_autorizada_) {
            RCLCPP_WARN(this->get_logger(), "[ENTREGA] No se inicia navegación: falta autorización de operador.");
            return;
        }

        iniciarNavegacion();
    }

    void iniciarNavegacion() {
        actualizarColaActiva();
        if (!cola_activa_) return;
        if (cola_activa_->empty()) return;

        if (!nav2Disponible()) {
            RCLCPP_WARN(this->get_logger(), "[NAV] Nav2 no disponible en este momento, reintenta más tarde.");
            state_ = IDLE;
            return;
        }

        if (modo_ == MODO_ENTREGA && !entrega_autorizada_) {
            RCLCPP_WARN(this->get_logger(), "[ENTREGA] Bloqueado: se requiere autorización antes de navegar.");
            state_ = IDLE;
            return;
        }

        if (!odom_recibida_) {
            publicarEstado("ERROR: No se mueve sin odometría previa.");
            state_ = ERROR;
            return;
        }

        auto objetivo = cola_activa_->front();

        NavigateToPose::Goal goal;
        goal.pose = objetivo;

        publicarEstado("Iniciando navegación...");
        state_ = NAVIGATING;
        start_nav_time_ = this->now();

        auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        options.feedback_callback =
            std::bind(&ControladorMesas::feedbackCallback, this, _1, _2);
        options.result_callback =
            std::bind(&ControladorMesas::resultCallback, this, _1);

        nav_client_->async_send_goal(goal, options);
    }

    void feedbackCallback(
        GoalHandleNav::SharedPtr handle,
        const std::shared_ptr<const NavigateToPose::Feedback>)
    {
        goal_handle_ = handle;
        publicarEstado("Navegación en progreso...");
    }

    void resultCallback(const GoalHandleNav::WrappedResult &result)
    {
        double duracion = (this->now() - start_nav_time_).seconds();

        if (timeout_habilitado_) {
            if (duracion > TIMEOUT_NAV &&
                result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                publicarEstado("Timeout excedido.");
                manejarFalloNavegacion("nav_timeout");
                return;
            }
        }

        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                state_ = WAITING_ODOM_CONFIRM;
                odom_stable_ = false;
                inicio_validacion_odom_ = this->now();
                break;

            case rclcpp_action::ResultCode::ABORTED:
                publicarEstado("Nav2 falló.");
                manejarFalloNavegacion("nav_aborted");
                break;

            case rclcpp_action::ResultCode::CANCELED:
                publicarEstado("Nav2 cancelado.");
                manejarFalloNavegacion("nav_canceled");
                break;

            default:
                publicarEstado("Error desconocido.");
                manejarFalloNavegacion("nav_error");
                break;
        }
    }

    void callbackOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_recibida_ = true;
        odom_last_time_ = this->now();

        ultima_odom_ = *msg;

        geometry_msgs::msg::PoseStamped odom_raw;
        odom_raw.header = msg->header;
        odom_raw.pose = msg->pose.pose;

        // Algunos drivers publican frame_id vacío o stamp 0; rellenamos para evitar fallos de TF.
        if (odom_raw.header.frame_id.empty()) {
            odom_raw.header.frame_id = "odom";
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "[ODOM] header.frame_id vacío, usando \"odom\" por defecto.");
        }
        if (odom_raw.header.stamp.sec == 0 && odom_raw.header.stamp.nanosec == 0) {
            odom_raw.header.stamp = this->now();
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "[ODOM] header.stamp en 0, usando tiempo actual para TF.");
        }

        geometry_msgs::msg::PoseStamped odom_mapa_tf;
        bool tiene_pose_mapa = poseEnMapa(odom_raw, odom_mapa_tf);

        if (!origen_establecido_) {
            if (!tiene_pose_mapa) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "[ORIGEN] Aún sin transform odom->map, esperando para fijar origen.");
            } else {
                double yaw_ini = yawFromQuat(odom_mapa_tf.pose.orientation);
                establecerOrigen(odom_mapa_tf.pose.position.x,
                                 odom_mapa_tf.pose.position.y,
                                 yaw_ini);
                RCLCPP_INFO(this->get_logger(),
                            "[ORIGEN] Origen inicial tomado en frame map (%.3f, %.3f, %.3f rad)",
                            origen_guardado_.x, origen_guardado_.y, origen_guardado_.theta);
                pub_origen_guardado_->publish(origen_guardado_);
            }
        }

        pub_odom_recibida_->publish(odom_raw);
        publicarEstadoOrigen(odom_mapa_tf, tiene_pose_mapa);

        actualizarColaActiva();
        if (!cola_activa_ || cola_activa_->empty()) return;

        auto objetivo = cola_activa_->front();

        geometry_msgs::msg::PoseStamped odom_mapa;
        if (!transformPose(odom_raw, objetivo.header.frame_id, odom_mapa, "ODOM_OBJETIVO")) {
            return;
        }

        pub_odom_mapa_->publish(odom_mapa);

        double dx = objetivo.pose.position.x - odom_mapa.pose.position.x;
        double dy = objetivo.pose.position.y - odom_mapa.pose.position.y;
        double dist = std::sqrt(dx*dx + dy*dy);

        std_msgs::msg::Float32 dist_msg;
        dist_msg.data = dist;
        pub_distancia_objetivo_->publish(dist_msg);

        double yaw_odom = yawFromQuat(odom_raw.pose.orientation);
        double yaw_robot = yawFromQuat(odom_mapa.pose.orientation);
        double yaw_goal = yawFromQuat(objetivo.pose.orientation);

        double raw_diff = yaw_robot - yaw_goal;
        double dyaw = std::atan2(std::sin(raw_diff), std::cos(raw_diff));

        std_msgs::msg::Float32 ymsg;
        ymsg.data = yaw_odom; pub_yaw_odom_->publish(ymsg);
        ymsg.data = yaw_robot; pub_yaw_mapa_->publish(ymsg);
        ymsg.data = yaw_goal; pub_yaw_objetivo_->publish(ymsg);
        ymsg.data = dyaw;     pub_yaw_diff_->publish(ymsg);

        if (state_ != WAITING_ODOM_CONFIRM) return;

        double tiempo_validando = (this->now() - inicio_validacion_odom_).seconds();

        if (tiempo_validando > TIMEOUT_VALIDACION) {
            publicarEstado("Validación falló — timeout.");
            publicarValidacion("validacion_timeout");
            auto objetivo_copia = objetivo;
            bool objetivo_es_origen = objetivoEsOrigen(objetivo_copia);
            objetivos_descartados_++;
            registrarDescartado(objetivo, "validacion_timeout");
            cola_activa_->pop();
            if (objetivo_es_origen) actualizarAutorizacionEntregaTrasObjetivo(objetivo_copia);
            state_ = IDLE;
            if (requerirPagoLuego(objetivo_copia)) return;
            encolarOrigenSiColaVacia();
            postTarea();
            return;
        }

        if (dist > REINICIO_MOVIMIENTO || std::fabs(dyaw) > REINICIO_YAW) {
            odom_stable_ = false;
            odom_stable_start_ = this->now();
        }

        bool pos_ok = dist < TOLERANCIA_POS;
        bool ang_ok = std::fabs(dyaw) < TOLERANCIA_ANG;

        if (!pos_ok || !ang_ok) {
            publicarValidacion(pos_ok ? "validacion_fallo_orientacion" : "validacion_fallo_posicion");
            odom_stable_ = false;
            return;
        }

        if (!odom_stable_) {
            odom_stable_ = true;
            odom_stable_start_ = this->now();
        }

        if ((this->now() - odom_stable_start_).seconds() >= TIEMPO_ESTABILIDAD) {
            publicarEstado("Llegada confirmada por odometría.");
            publicarValidacion("validacion_ok");
            auto objetivo_copia = objetivo;
            bool objetivo_es_origen = objetivoEsOrigen(objetivo_copia);
            cola_activa_->pop();
            if (objetivo_es_origen) actualizarAutorizacionEntregaTrasObjetivo(objetivo_copia);
            state_ = IDLE;
            if (requerirPagoLuego(objetivo_copia)) return;
            encolarOrigenSiColaVacia();
            postTarea();
        }
    }

    void postTarea() {
        if (automatico_activo_) avanzarModoAutomatico();
        else intentarIniciar();
    }

    std::string modoActualString() const {
        if (automatico_activo_) return "auto";
        switch (modo_) {
            case MODO_PEDIDO: return "pedido";
            case MODO_ENTREGA: return "entrega";
            case MODO_PAGO: return "pago";
            default: return "pedido";
        }
    }

    void publicarModoActual() {
        std_msgs::msg::String m;
        m.data = modoActualString();
        pub_modo_actual_->publish(m);
    }

    void registrarDescartado(const geometry_msgs::msg::PoseStamped &pose, const std::string &motivo) {
        cola_descartados_.push(ObjetivoDescartado{pose, motivo});
    }

    std::string listaDescartados() const {
        std::queue<ObjetivoDescartado> copia = cola_descartados_;
        std::stringstream ss;
        ss << "Descartados: [";
        bool first = true;
        while (!copia.empty()) {
            const auto &d = copia.front();
            if (!first) ss << ", ";
            ss << "(" << d.pose.pose.position.x << ", " << d.pose.pose.position.y << ")";
            if (!d.motivo.empty()) ss << " motivo:" << d.motivo;
            first = false;
            copia.pop();
        }
        ss << "]";
        return ss.str();
    }

    void establecerOrigen(double x, double y, double theta) {
        origen_guardado_.x = x;
        origen_guardado_.y = y;
        origen_guardado_.theta = normalizarAngulo(theta);
        origen_establecido_ = true;
        pub_origen_guardado_->publish(origen_guardado_);
        verificacion_origen_publicada_ = false; // forzar publicación del nuevo estado
    }

    void publicarEstadoOrigen(const geometry_msgs::msg::PoseStamped &odom_mapa, bool tiene_pose_mapa) {
        bool en_origen = false;
        if (origen_establecido_ && tiene_pose_mapa) {
            double dx0 = odom_mapa.pose.position.x - origen_guardado_.x;
            double dy0 = odom_mapa.pose.position.y - origen_guardado_.y;
            double dist0 = std::sqrt(dx0*dx0 + dy0*dy0);
            double yaw_actual = yawFromQuat(odom_mapa.pose.orientation);
            double dtheta = normalizarAngulo(yaw_actual - origen_guardado_.theta);
            bool pos_ok = dist0 < TOLERANCIA_POS;
            bool ang_ok = std::fabs(dtheta) < TOLERANCIA_ANG;
            en_origen = pos_ok && ang_ok;
        }
        if (origen_establecido_) {
            pub_origen_guardado_->publish(origen_guardado_);
        }
        if (!verificacion_origen_publicada_ || en_origen != ultimo_verificacion_origen_) {
            std_msgs::msg::Bool verif;
            verif.data = en_origen;
            pub_verificacion_origen_->publish(verif);
            verificacion_origen_publicada_ = true;
            ultimo_verificacion_origen_ = en_origen;
        }
        robot_en_origen_ = en_origen;
    }

    void encolarOrigenSiColaVacia() {
        actualizarColaActiva();
        if (!origen_establecido_ || !cola_activa_) return;
        if (!cola_activa_->empty()) return;
        if (robot_en_origen_) return;

        geometry_msgs::msg::PoseStamped origen_ps;
        origen_ps.header.frame_id = FRAME_MAP;
        origen_ps.header.stamp = this->now();
        origen_ps.pose.position.x = origen_guardado_.x;
        origen_ps.pose.position.y = origen_guardado_.y;
        origen_ps.pose.position.z = 0.0;
        origen_ps.pose.orientation = quatFromYaw(origen_guardado_.theta);

        cola_activa_->push(origen_ps);
        RCLCPP_INFO(this->get_logger(), "[ORIGEN] Cola vacía: se encola regreso automático al origen.");
    }

    bool requerirPagoLuego(const geometry_msgs::msg::PoseStamped &objetivo) {
        if (modo_ != MODO_PAGO) return false;
        if (objetivoEsOrigen(objetivo)) return false;
        esperando_confirmacion_pago_ = true;
        publicarEstado("Esperando confirmación de pago en topic pagado.");
        publicarValidacion("esperando_pago");
        return true;
    }

    bool objetivoEsOrigen(const geometry_msgs::msg::PoseStamped &obj) const {
        if (!origen_establecido_) return false;
        double dx = obj.pose.position.x - origen_guardado_.x;
        double dy = obj.pose.position.y - origen_guardado_.y;
        double dist = std::sqrt(dx*dx + dy*dy);
        double yaw_obj = yawFromQuat(obj.pose.orientation);
        double dyaw = normalizarAngulo(yaw_obj - origen_guardado_.theta);
        bool pos_ok = dist < TOLERANCIA_POS;
        bool ang_ok = std::fabs(dyaw) < TOLERANCIA_ANG;
        return pos_ok && ang_ok;
    }

    void actualizarAutorizacionEntregaTrasObjetivo(const geometry_msgs::msg::PoseStamped &obj) {
        if (modo_ == MODO_ENTREGA && objetivoEsOrigen(obj)) {
            entrega_autorizada_ = false;
        }
    }

    void manejarFalloNavegacion(const std::string &motivo) {
        actualizarColaActiva();
        geometry_msgs::msg::PoseStamped objetivo_copia;
        bool tiene_objetivo = false;
        if (cola_activa_ && !cola_activa_->empty()) {
            objetivo_copia = cola_activa_->front();
            tiene_objetivo = true;
            registrarDescartado(cola_activa_->front(), motivo);
            cola_activa_->pop();
            objetivos_descartados_++;
            if (objetivoEsOrigen(objetivo_copia)) actualizarAutorizacionEntregaTrasObjetivo(objetivo_copia);
        }
        state_ = IDLE;
        if (tiene_objetivo && requerirPagoLuego(objetivo_copia)) return;
        encolarOrigenSiColaVacia();
        postTarea();
    }

    void avanzarModoAutomatico() {
        if (modo_ == MODO_PEDIDO) {
            if (!cola_pedidos_.empty()) { iniciarNavegacion(); return; }
            modo_ = MODO_ENTREGA;
            publicarModoActual();
            entrega_autorizada_ = false;
        }

        if (modo_ == MODO_ENTREGA) {
            if (!cola_entregas_.empty()) { iniciarNavegacion(); return; }
            modo_ = MODO_PAGO;
            publicarModoActual();
        }

        if (modo_ == MODO_PAGO) {
            if (!cola_pagos_.empty()) { iniciarNavegacion(); return; }
            automatico_activo_ = false;
            publicarModoActual();
            return;
        }

        intentarIniciar();
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto nodo = std::make_shared<ControladorMesas>();
    rclcpp::spin(nodo);
    rclcpp::shutdown();
    return 0;
}
