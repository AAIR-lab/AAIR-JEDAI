function maximizeBlock(event) {

    block = workspace.getBlockById(event.blockId)
    block.setCollapsed(false)
}

function minimizeBlock(event) {

    block = event.getSourceBlock()
    block.setCollapsed(true)
}

function minimizeAllBlocks() {

    blocks = workspace.getAllBlocks()
    for (i = 0; i < blocks.length; i++) {

        blocks[i].setCollapsed(true)
    }
}

function maximizeAllBlocks() {

    blocks = workspace.getAllBlocks()
    for (i = 0; i < blocks.length; i++) {

        blocks[i].setCollapsed(false)
    }
}

function generate_action_block(action, objects, semantics, typesToSupertypes, color) {

    const inputsInline = false;
    const blockName = action.name + "_block";
    const params = action.params;

    // TODO handle missing semantics file
    const actionSemantics = semantics["actions"].find(a => a["name"] === action.name);
    const actionNameField = actionSemantics["display"] + "                                   ⓘ ";

    Blockly.Blocks[blockName] = {
        init: function () {
            // add the action name
            this.appendDummyInput()
                .appendField(actionNameField)
                .appendField(new Blockly.FieldImage("/static/roblocks/images/minimize_icon.jpg", 20, 20, null, minimizeBlock));

            // add all the parameters
            // have to keep track of which parameter goes where in the PDDL
            const paramNames = Array(params.length);
            for (let paramDetails of actionSemantics["parameters"]) {
                const position = paramDetails["index"];
                const param = params[position];
                // `labelFieldName` is the text that shows up on the block for this parameter
                const labelFieldName = paramDetails["display"] + ":";

                selected_objects = objects.filter(o => isOfType(o.type, param.type, typesToSupertypes))
                    .map(o => [o.name, o.name])
                
                // Remove the "starting_point" from 
                // a. All actions that are not the move action 
                // b. The "?to" location of the move action.
                if (semantics["domain"] == "cafeworld" ) {

                    index = -1
                    for (idx = 0; idx < selected_objects.length; idx++) {
                        if (selected_objects[idx][0] == "starting_point") {

                            index = idx
                            break
                        }
                    }

                    if (index > -1 
                        && (action.name != "move" || paramDetails.index == 2)) {

                        selected_objects.splice(index, 1)
                    }
                } 

                this.appendDummyInput()
                    .appendField(labelFieldName)
                    .appendField(new Blockly.FieldDropdown(
                        selected_objects,
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
    let toolbox=document.getElementById('Actions');
    if (document.getElementById(actionSemantics.category) == null){
        let element = document.createElement('category');
        element.id = actionSemantics.category;
        element.setAttribute('name', actionSemantics.category);
        toolbox.append(element);
    }
    $('[name="'+actionSemantics.category+'"]').append(xml);
}

function generateActionBlocks(actions, objects, semantics, typesToSupertypes) {
    const promises = [];
    const inputsInline = false;

    const blockGenerator = (index, color) => new Promise((resolve, reject) => {
        action = actions[index];
        
        generate_action_block(action, objects, semantics, typesToSupertypes, color);
        
    });

    const blockGenerator2 = (color) => new Promise((resolve, reject) => {
        const blockName = "loops"+ "_block";
        Blockly.Blocks[blockName] = {
            init: function() {
                // Add the "while" label
                this.appendDummyInput()
                    .appendField("while");
        
                // Add a boolean input for the loop condition
                this.appendValueInput("CONDITION")
                    .setCheck("Boolean")
                    .appendField("condition");
        
                // Add a statement input for the loop body
                this.appendStatementInput("DO")
                    .setCheck(null)
                    .appendField("do");
        
                this.setInputsInline(inputsInline);
                this.setPreviousStatement(true, null);
                this.setNextStatement(true, null);
                this.setColour(color);
                this.setHelpUrl("");
            }
        };        
        const xml = '<block type="' + blockName + '"></block>';
        // TODO handle category names better
        let toolbox=document.getElementById('Loops');
        let element = document.createElement('category');
        element.id = "loops";
        element.setAttribute('name',"loops");
        toolbox.append(element);
        $('[name="'+"loops"+'"]').append(xml);

    });

    const blockGenerator3 = (color) => new Promise((resolve, reject) => {
        const blockName = "goals"+ "_block";
        Blockly.Blocks[blockName] = {
            init: function() {
                // Add the "Goal Condition Not Met" label
                this.appendDummyInput()
                    .appendField("Goal Condition Not Met");
        
                // Set the output type to Boolean so it can fit in the condition slot
                this.setOutput(true, "Boolean");
        
                this.setColour(color);
                this.setTooltip("Represents a goal condition that's not met.");
                this.setHelpUrl("");
 
                // this.setInputsInline(inputsInline);
                // this.setPreviousStatement(true, null);
                // this.setNextStatement(true, null);
                this.setColour(color);
                this.setHelpUrl("");
                 
            }
        };
        const xml = '<block type="' + blockName + '"></block>';
        // TODO handle category names better
        let toolbox=document.getElementById('Goals');
        let element = document.createElement('category');
        element.id = "goals";
        element.setAttribute('name',"goals");
        toolbox.append(element);
        $('[name="'+"goals"+'"]').append(xml);

    });



    const step = 90 / (actions.length*0.5);
    let color = 90;
    for(let i = 0; i < actions.length; i++){
        color += step;
        promises.push(blockGenerator(i, color));
    }

    promises.push(blockGenerator2('#33AACC'))
    promises.push(blockGenerator3("#FFAA33"))
    Promise.all(promises);
}

// function generateActionBlocks_predicates(preds, objects, semantics, typesToSupertypes) {
//     const inputsInline = false;
//     const promises = [];
//     console.log("Length of predicates = ",semantics['predicates'].length);
//     console.log("preds = ",preds)
//     const blockGenerator = (index, color) => new Promise((resolve, reject) => {
//         // const action = actions[index];
//         const predicate = semantics['predicates'][index];

//         // const blockName = action.name + "_block";
//         const blockName = predicate.name + "_block";

//         // TODO handle missing semantics file
//         // const actionSemantics = semantics["actions"].find(a => a["name"] === action.name);
//         const predicateSemantics = semantics["predicates"][index];

//         // `actionNameField` is the text that shows up on the block
//         const predicateNameField = predicateSemantics["display"].split("%")[0] + "...";
//         // `blockName` is the name of the block behind the scenes

//         // const params = action.params;
//         const params = preds[predicate.name];
        
//         Blockly.Blocks[blockName] = {
//             init: function () {
//                 // add the action name
//                 this.appendDummyInput()
//                     .appendField(predicateNameField);

//                 // add all the parameters
//                 // have to keep track of which parameter goes where in the PDDL

//                 // for (let paramDetails of actionSemantics["parameters"]) {

//                 if(params.length == 2){
//                     const subject = params[predicateSemantics['subjectIndex']]
//                     const object = params[1- predicateSemantics['subjectIndex']]
                    
//                     this.appendDummyInput()
//                     .appendField(subject)
//                     .appendField(new Blockly.FieldDropdown(
//                         objects.filter(
//                             o => isOfType(o.type, subject, typesToSupertypes)
//                         ).map(o => [o.name, o.name])
//                     ), subject).appendField(" is ").appendField(predicateSemantics["display"].split("%")[0])
//                     .appendField(object).appendField(new Blockly.FieldDropdown(
//                         objects.filter(
//                             o => isOfType(o.type, object, typesToSupertypes)
//                         ).map(o => [o.name, o.name])
//                     ), object);
                    
//                 }
//                 else{
//                     const subject = params[predicateSemantics['subjectIndex']]                    
//                     this.appendDummyInput()
//                     .appendField(subject)
//                     .appendField(new Blockly.FieldDropdown(
//                         objects.filter(
//                             o => isOfType(o.type, subject, typesToSupertypes)
//                         ).map(o => [o.name, o.name])
//                     ), subject).appendField(" is ").appendField(predicateSemantics["display"].split("%")[0])
//                 }



//                 // use `data` field to store the ordering of the param names as in the PDDL
//                 this.data = params.join(" ");

//                 // settings
//                 this.setInputsInline(inputsInline);
//                 this.setPreviousStatement(true, null);
//                 this.setNextStatement(true, null);
//                 this.setColour(color);
//                 this.setHelpUrl("");
//             }
//         };
//         const xml = '<block type="' + blockName + '"></block>';
//         // TODO handle category names better
//         let toolbox=document.getElementById('Predicates');
//         console.log("TOolbox = ",toolbox);
//         if (document.getElementById(predicateSemantics.name) == null){
//             console.log("INSIDE~~")
//             let element = document.createElement('category');
//             element.id = predicateSemantics.name;
//             element.setAttribute('name', predicateSemantics.name);
//             toolbox.append(element);
//         }
//         $('[name="'+predicateSemantics.name+'"]').append(xml);

//     });

//     const step = 90 / (semantics['predicates'].length*0.5);
//     let color = 90;
//     for(let i = 0; i < semantics['predicates'].length; i++){
//         color += step;
//         promises.push(blockGenerator(i, color));
//     }
//     console.log("Length of promieses = ",promises.length)
//     Promise.all(promises);
// }



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




