/*
 * Main node for robocupathome_pnp
 */

#include "RCHPNPAS.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robocupathome_pnpas");

    ros::NodeHandle node;
    
    RCHPNPActionServer rchpnpas(node);
    rchpnpas.start();
    ros::spin();

    return 0;
}

