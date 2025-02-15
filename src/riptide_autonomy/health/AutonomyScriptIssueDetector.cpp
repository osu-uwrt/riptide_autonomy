#include "riptide_autonomy/autonomy_health.hpp"


AutonomyScriptIssueDetector::AutonomyScriptIssueDetector(
    tinyxml2::XMLElement *node,
    const std::string& file,
    std::shared_ptr<const BT::BehaviorTreeFactory> factory,
    const NodeManifests& palette,
    const std::vector<std::string>& blackboardDefinitions)
 : AutonomyNodeIssueDetector(node, file, factory, palette, blackboardDefinitions) {}

HealthError AutonomyScriptIssueDetector::detect()
{
    const char *name = _node->Name();
    if(std::string(name) != "Script")
    {
        return HealthError(true, "Cannot analyze script because provided node is not a script");
    }
    
    const char *code = _node->Attribute("code");
    if(!code)
    {
        addIssue(
            std::make_shared<UnfixableAutonomyIssue>(
                ISSUE_ERROR,
                _fileName,
                _node->GetLineNum(),
                "ScriptError",
                "Script is missing code"));
            
        return HealthError(true, "Aborted due to previous errors");
    }

    BT::Result res = BT::ValidateScript(code);
    if(!res)
    {
        addIssue(
            std::make_shared<UnfixableAutonomyIssue>(
                ISSUE_ERROR,
                _fileName,
                _node->GetLineNum(),
                "ScriptSyntaxError",
                "Parse error: " + res.error()));
    
        return HealthError(true, "Aborted due to previous errors");
    }

    //now need to simulate the script in order to populate blackboard pointers. 
    BT::Ast::Environment env;
    env.vars = BT::Blackboard::create();
    for(std::string bbDef : _blackboardDefs)
    {
        env.vars->set<double>(bbDef, 1);
    }

    //NOTE: for now enums are not supported because they are not made available by the factory

    try
    {
        auto execRes = BT::ParseScriptAndExecute(env, code);
        if(!execRes)
        {
            addIssue(
                std::make_shared<UnfixableAutonomyIssue>(
                    ISSUE_ERROR,
                    _fileName,
                    _node->GetLineNum(),
                    "ScriptRuntimeError",
                    "Runtime Error: " + res.error()));
            
            return HealthError(true, "Aborted due to previous errors");
        }
    } catch(BT::RuntimeError& ex)
    {
        addIssue(
            std::make_shared<UnfixableAutonomyIssue>(
                ISSUE_ERROR,
                _fileName,
                _node->GetLineNum(),
                "ScriptRuntimeError",
                "Runtime Error: " + std::string(ex.what())));
        
        return HealthError(true, "Aborted due to previous errors");
    }

    //now assuming that the script succeeded, note any new blackboard definitions
    std::vector<BT::StringView> newBBKeys = env.vars->getKeys();
    for(size_t i = 0; i < newBBKeys.size(); i++)
    {
        std::string newKey = std::string(newBBKeys[i]);
        if(std::find(_blackboardDefs.begin(), _blackboardDefs.end(), newKey) == _blackboardDefs.end())
        {
            _blackboardDefs.push_back(newKey);
        }
    }

    return HealthError(false, "");
}
