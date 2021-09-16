#include "states.h"

/**
 * Source file for the Util class.
 */
using namespace BT;

/**
 * Parses a string input "x,y,z,w" into a Quaternion.
 */
geometry_msgs::Quaternion Util::quaternionFromString(std::string str) {
    std::vector<StringView> parts = splitString(str, ',');
    if(parts.size() == 4) {

        geometry_msgs::Quaternion quaternion;
        quaternion.x = convertFromString<double>(parts[0].data());
        quaternion.y = convertFromString<double>(parts[1].data());
        quaternion.z = convertFromString<double>(parts[2].data());
        quaternion.w = convertFromString<double>(parts[3].data());


        return quaternion;
    }

    throw RuntimeError("quaternionFromString(BT::Stringview): Invalid Input! Expected string with format x,y,z,w!");
}