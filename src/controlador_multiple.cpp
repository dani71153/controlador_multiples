#include <memory>
#include <queue>
#include <vector>
#include <sstream>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/exceptions.h"

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

    enum State { IDLE, NAVIGATING, WAITING_ODOM_CONFIRM, ERROR };
    enum ModoOperacion { MODO_PEDIDO, MODO_ENTREGA, MODO_PAGO, MODO_AUTO };

    ControladorMesas() : Node("controlador_mesas")
    {
        modo_ = MODO_PEDIDO;
        automatico_activo_ = false;
        ultimo_estado_ = "IDLE";
        ultimo_validacion_ = "";
        nav2_activo_ = false;

        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
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

        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 50, std::bind(&ControladorMesas::callbackOdom, this, _1));

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

        sub_nav2_estado_ = this->create_subscription<std_msgs::msg::Bool>(
            "/nav2_lifecycle_manager_navigation/is_active",
            10,
            std::bind(&ControladorMesas::callbackNav2Estado, this, _1)
        );

        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        state_ = IDLE;

        timer_publicacion_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ControladorMesas::tickPublicacion, this)
        );

        timer_chequeo_nav2_ = this->create_wall_timer(
            std::chrono::milliseconds(400),
            [this]() {
                RCLCPP_INFO(this->get_logger(), "=== ANALISIS INICIAL NAV2 ===");
                bool ok = nav2Disponible();
                if (ok)
                    RCLCPP_INFO(this->get_logger(), "[INIT] Nav2 DISPONIBLE al inicio.");
                else
                    RCLCPP_WARN(this->get_logger(), "[INIT] Nav2 NO disponible al inicio.");
                RCLCPP_INFO(this->get_logger(), "=== FIN ANALISIS NAV2 ===");
                timer_chequeo_nav2_.reset();
            }
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

        RCLCPP_INFO(this->get_logger(), "[INIT] Subscribers creados:");
        RCLCPP_INFO(this->get_logger(), "   - /odom");
        RCLCPP_INFO(this->get_logger(), "   - entradas_pedidos");
        RCLCPP_INFO(this->get_logger(), "   - entradas_entregas");
        RCLCPP_INFO(this->get_logger(), "   - entradas_pagos");
        RCLCPP_INFO(this->get_logger(), "   - controlador_mesas/limpiar_cola");
        RCLCPP_INFO(this->get_logger(), "   - controlador_mesas/timeout_control");
        RCLCPP_INFO(this->get_logger(), "   - controlador_mesas/reiniciar");
        RCLCPP_INFO(this->get_logger(), "   - controlador_mesas/cambiar_modo");
        RCLCPP_INFO(this->get_logger(), "   - /nav2_lifecycle_manager_navigation/is_active");

        RCLCPP_INFO(this->get_logger(), "[INIT] Sistema de TF listo.");
        RCLCPP_INFO(this->get_logger(), "[INIT] Action Client navigate_to_pose inicializado.");

        bool nav2_now = nav2Disponible();
        RCLCPP_INFO(this->get_logger(), "[INIT] Estado Nav2: %s", nav2_now ? "Disponible" : "No disponible");

        RCLCPP_INFO(this->get_logger(), "[INIT] Estado inicial del nodo: IDLE");
        RCLCPP_INFO(this->get_logger(), "=== CONTROLADOR DE MESAS LISTO ===");

    }

private:
    ModoOperacion modo_;
    bool automatico_activo_;
    State state_;

    bool nav2_activo_;

    std::queue<geometry_msgs::msg::PoseStamped> cola_pedidos_;
    std::queue<geometry_msgs::msg::PoseStamped> cola_entregas_;
    std::queue<geometry_msgs::msg::PoseStamped> cola_pagos_;
    std::queue<geometry_msgs::msg::PoseStamped>* cola_activa_ = nullptr;

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

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pedidos_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_entregas_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pagos_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_limpiar_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_timeout_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_reiniciar_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_modo_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_nav2_estado_;

    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    GoalHandleNav::SharedPtr goal_handle_;

    rclcpp::TimerBase::SharedPtr timer_publicacion_;
    rclcpp::TimerBase::SharedPtr timer_chequeo_nav2_;

    bool nav2Disponible()
    {
        RCLCPP_INFO(this->get_logger(), "[NAV2 CHECK] Iniciando verificación...");
        bool action_ok = nav_client_->wait_for_action_server(std::chrono::milliseconds(300));
        auto topics = this->get_topic_names_and_types();
        bool lifecycle_ok = false;
        for (auto &t : topics)
            if (t.first == "/nav2_lifecycle_manager_navigation/is_active")
                lifecycle_ok = true;
        return action_ok && lifecycle_ok;
    }

    double yawFromQuat(const geometry_msgs::msg::Quaternion &q) {
        double siny = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z);
        return std::atan2(siny, cosy);
    }

    void publicarEstado(const std::string &s) {
        ultimo_estado_ = s;
        std_msgs::msg::String m; m.data = s;
        pub_estado_->publish(m);
        RCLCPP_INFO(this->get_logger(), "%s", s.c_str());
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
    }

    void callbackNav2Estado(const std_msgs::msg::Bool::SharedPtr msg)
    {
        nav2_activo_ = msg->data;
        publicarEstado(nav2_activo_ ? "Nav2 ACTIVO." : "Nav2 INACTIVO.");
    }

    void callbackPedido(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        cola_pedidos_.push(*msg);
        publicarEstado("Pedido añadido.");
        intentarIniciar();
    }

    void callbackEntrega(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        cola_entregas_.push(*msg);
        publicarEstado("Entrega añadida.");
        intentarIniciar();
    }

    void callbackPago(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        cola_pagos_.push(*msg);
        publicarEstado("Pago añadido.");
        intentarIniciar();
    }

    void callbackTimeoutControl(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "on") {
            timeout_habilitado_ = true;
            publicarEstado("Timeout habilitado.");
        }
        else if (msg->data == "off") {
            timeout_habilitado_ = false;
            publicarEstado("Timeout deshabilitado.");
        }
        else {
            publicarEstado("Comando timeout inválido.");
        }
    }

    void callbackCambiarModo(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "pedido") {
            modo_ = MODO_PEDIDO;
            automatico_activo_ = false;
            publicarEstado("Modo PEDIDO");
        }
        else if (msg->data == "entrega") {
            modo_ = MODO_ENTREGA;
            automatico_activo_ = false;
            publicarEstado("Modo ENTREGA");
        }
        else if (msg->data == "pago") {
            modo_ = MODO_PAGO;
            automatico_activo_ = false;
            publicarEstado("Modo PAGO");
        }
        else if (msg->data == "auto") {
            modo_ = MODO_PEDIDO;
            automatico_activo_ = true;
            publicarEstado("Modo AUTO activado");
        }
        else {
            publicarEstado("Modo inválido.");
        }

        intentarIniciar();
    }

    void callbackLimpiarCola(const std_msgs::msg::String::SharedPtr) {
        if (cola_activa_) while (!cola_activa_->empty()) cola_activa_->pop();
        publicarEstado("Cola actual eliminada.");

        if (state_ == NAVIGATING && goal_handle_) {
            nav_client_->async_cancel_goal(goal_handle_);
            publicarEstado("Navegación cancelada.");
        }

        state_ = IDLE;
    }

    void callbackReiniciar(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "reiniciar") reiniciarNodo();
    }

    void reiniciarNodo() {
        publicarEstado("Reiniciando...");

        if (goal_handle_) {
            try {
                auto st = goal_handle_->get_status();
                if (st == rclcpp_action::GoalStatus::STATUS_ACCEPTED ||
                    st == rclcpp_action::GoalStatus::STATUS_EXECUTING)
                {
                    nav_client_->async_cancel_goal(goal_handle_);
                    publicarEstado("Navegación cancelada.");
                }
            } catch (...) {}
        }

        while (!cola_pedidos_.empty()) cola_pedidos_.pop();
        while (!cola_entregas_.empty()) cola_entregas_.pop();
        while (!cola_pagos_.empty()) cola_pagos_.pop();

        timeout_habilitado_ = false;
        odom_stable_ = false;
        objetivos_descartados_ = 0;

        goal_handle_.reset();
        state_ = IDLE;

        publicarEstado("Reinicio completado.");
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
            ss << "Mesa actual: (" << m.pose.position.x << ", " << m.pose.position.y << ")";
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
        RCLCPP_INFO(this->get_logger(), "[FLOW] intentarIniciar() llamado. state=%d", state_);

        if (state_ != IDLE) {
            RCLCPP_INFO(this->get_logger(), "[FLOW] No inicia porque state != IDLE");
            return;
        }

        actualizarColaActiva();

        if (!cola_activa_) {
            RCLCPP_WARN(this->get_logger(), "[FLOW] Cola activa es nullptr.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "[FLOW] Cola activa size=%zu", cola_activa_->size());

        if (!cola_activa_->empty()) {
            RCLCPP_INFO(this->get_logger(), "[FLOW] Cola NO vacía. Llamando iniciarNavegacion().");
            iniciarNavegacion();
        } else {
            RCLCPP_INFO(this->get_logger(), "[FLOW] Cola vacía. No se inicia navegación.");
        }
    }

    void iniciarNavegacion() {
        RCLCPP_INFO(this->get_logger(), "[FLOW] iniciarNavegacion() llamado.");

        actualizarColaActiva();
        if (!cola_activa_) {
            RCLCPP_WARN(this->get_logger(), "[FLOW] iniciarNavegacion(): cola_activa_ es nullptr.");
            return;
        }

        if (cola_activa_->empty()) {
            RCLCPP_INFO(this->get_logger(), "[FLOW] iniciarNavegacion(): cola vacía, no hay objetivo.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "[FLOW] iniciarNavegacion(): llamando nav2Disponible().");
        if (!nav2Disponible()) {
            publicarEstado("ERROR: Nav2 NO está ejecutándose.");
            state_ = IDLE;
            RCLCPP_WARN(this->get_logger(), "[FLOW] iniciarNavegacion(): nav2Disponible() = false, abortando.");
            return;
        }

        auto objetivo = cola_activa_->front();
        RCLCPP_INFO(this->get_logger(),
                    "[FLOW] iniciarNavegacion(): objetivo en (%.3f, %.3f, frame=%s)",
                    objetivo.pose.position.x,
                    objetivo.pose.position.y,
                    objetivo.header.frame_id.c_str());

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

        RCLCPP_INFO(this->get_logger(), "[FLOW] iniciarNavegacion(): enviando goal a Nav2...");
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
                state_ = ERROR;
                return;
            }
        }

        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                publicarEstado("Nav2 éxito — validando...");
                state_ = WAITING_ODOM_CONFIRM;
                odom_stable_ = false;
                inicio_validacion_odom_ = this->now();
                break;

            case rclcpp_action::ResultCode::ABORTED:
                publicarEstado("Nav2 falló.");
                state_ = ERROR;
                break;

            case rclcpp_action::ResultCode::CANCELED:
                publicarEstado("Nav2 cancelado.");
                state_ = ERROR;
                break;

            default:
                publicarEstado("Error desconocido.");
                state_ = ERROR;
                break;
        }
    }

    void callbackOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        ultima_odom_ = *msg;

        geometry_msgs::msg::PoseStamped odom_raw;
        odom_raw.header = msg->header;
        odom_raw.pose = msg->pose.pose;

        pub_odom_recibida_->publish(odom_raw);

        actualizarColaActiva();
        if (!cola_activa_ || cola_activa_->empty()) return;

        auto objetivo = cola_activa_->front();

        geometry_msgs::msg::PoseStamped odom_mapa;
        try {
            tf_buffer_->transform(odom_raw, odom_mapa, objetivo.header.frame_id);
        } catch (...) { return; }

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

            objetivos_descartados_++;

            cola_activa_->pop();
            state_ = IDLE;

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

            cola_activa_->pop();
            state_ = IDLE;

            postTarea();
        }
    }

    void postTarea() {
        if (automatico_activo_) {
            avanzarModoAutomatico();
        } else {
            intentarIniciar();
        }
    }

    void avanzarModoAutomatico() {
        if (modo_ == MODO_PEDIDO) {
            if (!cola_pedidos_.empty()) { iniciarNavegacion(); return; }
            modo_ = MODO_ENTREGA;
            publicarEstado("AUTO → ENTREGA");
        }

        if (modo_ == MODO_ENTREGA) {
            if (!cola_entregas_.empty()) { iniciarNavegacion(); return; }
            modo_ = MODO_PAGO;
            publicarEstado("AUTO → PAGO");
        }

        if (modo_ == MODO_PAGO) {
            if (!cola_pagos_.empty()) { iniciarNavegacion(); return; }
            publicarEstado("AUTO → Ronda completa");
            automatico_activo_ = false;
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
