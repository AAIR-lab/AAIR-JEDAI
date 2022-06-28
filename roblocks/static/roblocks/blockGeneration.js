function generateActionBlocks(actions, objects, semantics, typesToSupertypes) {
    const inputsInline = false;
    const promises = [];
    const blockGenerator = (index, color) => new Promise((resolve, reject) => {
        const action = actions[index];
        const blockName = action.name + "_block";
        // TODO handle missing semantics file
        const actionSemantics = semantics["actions"].find(a => a["name"] === action.name);
        // `actionNameField` is the text that shows up on the block
        const actionNameField = actionSemantics["display"] + "...";
        // `blockName` is the name of the block behind the scenes
        const params = action.params;
        Blockly.Blocks[blockName] = {
            init: function () {
                // add the action name
                this.appendDummyInput()
                    .appendField(actionNameField);

                // add all the parameters
                // have to keep track of which parameter goes where in the PDDL
                const paramNames = Array(params.length);
                for (let paramDetails of actionSemantics["parameters"]) {
                    const position = paramDetails["index"];
                    const param = params[position];
                    // `labelFieldName` is the text that shows up on the block for this parameter
                    const labelFieldName = paramDetails["display"] + ":";

                    this.appendDummyInput()
                        .appendField(labelFieldName)
                        .appendField(new Blockly.FieldDropdown(
                            objects.filter(
                                o => isOfType(o.type, param.type, typesToSupertypes)
                            ).map(o => [o.name, o.name])
                        ), param.name);

                    paramNames[position] = param.name;
                }
                // use `data` field to store the ordering of the param names as in the PDDL
                this.data = paramNames.join(" ");

                // settings
                this.setInputsInline(inputsInline);
                this.setPreviousStatement(true, null);
                this.setNextStatement(true, null);
                this.setColour(color);
                this.setTooltip(actionSemantics["tooltip"]);
                this.setHelpUrl("");
            }
        };
        const xml = '<block type="' + blockName + '"></block>';
        // TODO handle category names better
        $('[name="Actions"]').append(xml);
    });
    const step = 90 / (actions.length*0.5);
    let color = 90;
    for(let i = 0; i < actions.length; i++){
        color += step;
        promises.push(blockGenerator(i, color));
    }
    Promise.all(promises);
}

function getColor(block,index,startColor,length){
    var  blockArr = { "red":"#f73727", "blue":"#4b6dd8", "green":"#0bb409", "gripper":"#545454" };
    var color = blockArr[block.split("_")[0]];
    if(color == null){
        var step = 270/length;
        color = 90 + step*index;
    }

    return color;
}

function isOfType(typeToCheck, typeToCheckAgainst, typesToSupertypes) {
    // check if the types are already an exact match
    if (typeToCheck === typeToCheckAgainst) {
        return true;
    }

    // check if any supertypes of the type to check are an exact match
    let roamingType = typeToCheck;
    while (typesToSupertypes[roamingType]) {
        roamingType = typesToSupertypes[roamingType];
        if (roamingType === typeToCheckAgainst) {
            return true;
        }
    }

    return false;
}
