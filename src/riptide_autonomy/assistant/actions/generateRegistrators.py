import os
from util import categorizedGlob, fileNameNoExt, createFileFromTemplate, info, autonomySrcLocation, autonomyIncludeLocation

def onGenerateRegistrators(args, autonomyRootLoc: str):
    location = args.directory
    
    #resolve general node file names    
    generalActionFiles, generalConditionFiles, generalDecoratorFiles = categorizedGlob(autonomyIncludeLocation(autonomyRootLoc), "*.hpp")
    
    #resolve simple node file names
    simpleActionFiles, simpleConditionFiles, simpleDecoratorFiles = categorizedGlob(autonomySrcLocation(autonomyRootLoc), "*.simple.cpp")
    
    #creates a single registrator
    def createRegistrator(generalFiles: 'list[str]', simpleFiles: 'list[str]', location: str):
        registrations = ""
        headers = ""
        for file in generalFiles:
            # perform general registration on all other hpps
            registrations += "    factory.registerNodeType<{0}>(\"{0}\");\n".format(fileNameNoExt(file))
            
            #figure out node type (its in the name between the second to last and last dots)
            lastDot = file.rfind(".")
            secondLastDot = file.rfind(".", 0, lastDot - 1)
                            
            #not checking dots because files need to have two to be globbed
            nodeType = file[secondLastDot + 1 : lastDot]
            
            nodeDir = ""
            if(nodeType == "btaction"):
                nodeDir = "bt_actions"
            elif(nodeType == "btcondition"):
                nodeDir = "bt_conditions"
            elif(nodeType == "btdecorator"):
                nodeDir = "bt_decorators"
            
            headers += "#include \"riptide_autonomy/{}/{}\"\n".format(nodeDir, os.path.basename(file))
        
        for file in simpleFiles:
            registrations += "    {0}::bulkRegister(factory);\n".format(fileNameNoExt(file))
        
        templatePath = "{}/assistant/templates/plugin_registrator_template".format(autonomySrcLocation(autonomyRootLoc))
        createFileFromTemplate(templatePath, location, [headers, registrations])
    
    createRegistrator(generalActionFiles, simpleActionFiles, "{}/registerActions.cpp".format(location))
    createRegistrator(generalConditionFiles, simpleConditionFiles, "{}/registerConditions.cpp".format(location))
    createRegistrator(generalDecoratorFiles, simpleDecoratorFiles, "{}/registerDecorators.cpp".format(location))
    
    info(args, "Generated registrators in directory {}".format(os.path.abspath(location)))

