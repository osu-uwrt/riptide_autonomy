#include "autonomy_test/autonomy_health_util_testing.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

void AutonomyHealthUtilTest::SetUp()
{
    ::testing::Test::SetUp();
    _factory = std::make_shared<BT::BehaviorTreeFactory>();
    registerPluginsForFactory(_factory, AUTONOMY_PKG_NAME);
}


void AutonomyHealthUtilTest::TearDown()
{
    //teardown operations here...
    _factory.reset();
    ::testing::Test::TearDown();
}


tinyxml2::XMLElement *AutonomyHealthUtilTest::walkTree(tinyxml2::XMLDocument& doc, const std::string& file, const std::vector<std::pair<std::string, int>>& path)
{
    //find the desired file in the share directory
    std::string 
        shareDir = ament_index_cpp::get_package_share_directory(AUTONOMY_PKG_NAME),
        fullPath = shareDir + "/health_test_trees/" + file;

    doc.LoadFile(fullPath.c_str());
    if(doc.Error())
    {
        std::cerr << "Failed to load doc" << file << " : " << doc.ErrorStr() << std::endl;
        return nullptr;
    }

    tinyxml2::XMLElement *elem = doc.RootElement();

    if(!elem)
    {
        std::cerr << "No root element for file " << fullPath << std::endl;
        return nullptr;
    }

    //navigate down tree in path
    for(std::pair<std::string, int> part : path)
    {
        //navigate across to correct sibling
        elem = elem->FirstChildElement(part.first.c_str());
        for(int i = 0; i < part.second; i++)
        {
            if(!elem)
            {
                std::cerr << "Ran out of siblings for " << part.first << " at index " << part.second << std::endl;
                break;
            }

            elem = elem->NextSiblingElement(part.first.c_str());
        }

        if(!elem)
        {
            std::cerr << "Cannot find occurrance " << part.second << " of tag " << part.first << " of file " << fullPath << std::endl;
            break;
        }
    }

    if(!elem)
    {
        std::string msg = "Could not find element at path: " + path[0].first;
        for(int i = 1; i < path.size(); i++)
        {
            msg += ", " + path[i].first;
        }

        std::cerr << msg << std::endl;
    }

    return elem;
}


void AutonomyHealthUtilTest::printIssuesIf(const AutonomyIssueDetector& detector, bool condition)
{

    if(condition)
    {
        std::vector<AutonomyIssue::Ptr> issues = detector.issues();
        std::string message = "UNEXPECTED: " + std::to_string(issues.size()) + " issues detected:\n";
        for(size_t i = 0; i < issues.size(); i++)
        {
            message += issues[i]->issue() + "\n";
        }

        std::cerr << message << std::endl;
    }
}


bool AutonomyHealthUtilTest::issueVectorContains(const std::vector<AutonomyIssue::Ptr>& vec, const std::string issueType)
{
    for(AutonomyIssue::Ptr iss : vec)
    {
        if(iss->type() == issueType)
        {
            return true;
        }
    }

    return false;
}
