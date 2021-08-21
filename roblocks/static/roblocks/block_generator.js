function generateActionBlocks(actions, objects, semantics) {
    const inputsInline = false;
    const promises = [];
    const generateBlocks = (index, color) => new Promise((resolve, reject) => {
        const action = actions[index];
        const blockName = action.name + "_block";
        // TODO this if statement is bad
        if (semantics) {
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
                                objects.filter(o => o.type === param.type).map(o => [o.name, o.name])
                            ), param.name);

                        paramNames[position] = param.name;
                    }
                    // use `data` field to access the ordering of the param names in the PDDL
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
        }
        else {
            // `actionNameField` is the text that shows up on the block
            const actionNameField = action.name;
            const params = action.params;
            Blockly.Blocks[blockName] = {
                init: function () {
                    // add the action name
                    this.appendDummyInput()
                        .appendField(actionNameField);

                    // add all the parameters
                    // have to keep track of which parameter goes where in the PDDL
                    const paramNames = Array(params.length);
                    for (let param of params) {
                        // `labelFieldName` is the text that shows up on the block for this parameter
                        const labelFieldName = param.name.substring(1);

                        this.appendDummyInput()
                            .appendField(labelFieldName)
                            .appendField(new Blockly.FieldDropdown(
                                objects.filter(o => o.type === param.type).map(o => [o.name, o.name])
                            ), param.name);
                        paramNames.push(param.name)
                    }
                    // use `data` field to access the ordering of the param names in the PDDL
                    this.data = paramNames.join(" ");

                    // settings
                    this.setInputsInline(inputsInline);
                    this.setPreviousStatement(true, null);
                    this.setNextStatement(true, null);
                    this.setColour(color);
                    this.setTooltip("");
                    this.setHelpUrl("");
                }
            };
        }
        const xml = '<block type="' + blockName + '"></block>';
        // TODO handle category names better
        $('[name="Actions"]').append(xml);
    });
    const step = 90 / actions.length;
    let color = 0;
    for(let i = 0; i < actions.length; i++){
        color += step;
        promises.push(generateBlocks(i, color));

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
