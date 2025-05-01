#include <modelec_strat/mission_manager.hpp>

namespace Modelec
{
    enum class State {
        INIT,               // Initialisation des capteurs, timers, etc.
        WAIT_START,         // Attente du feu vert
        SELECT_MISSION,     // Choix intelligent de la prochaine mission

        RETURN_HOME,        // Retour zone de fin
        FAILSAFE,           // En cas d'erreur bloquante
        STOP,                // Fin du match

        GO_TO_OBJECTIVE,    // Navigation vers la cible
        EXECUTE_MISSION,    // Exécution de l'action
    };

    MissionManager::MissionManager()
    {
    }

    MissionManager::MissionManager(const rclcpp::Node::SharedPtr& node) : node_(node)
    {
    }

    rclcpp::Node::SharedPtr MissionManager::getNode() const
    {
        return node_;
    }
}
