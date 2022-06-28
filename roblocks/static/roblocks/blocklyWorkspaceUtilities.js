var workspace;

/**
 * Initializes the Blockly workspace.
 * */
function initializeBlocks(){
    let loader = document.getElementById("submission-loader");
    loader.style.display = "none";
    workspace = Blockly.inject('blocklyDiv',
        {
            toolbox: document.getElementById('toolbox'),
            zoom: {
                controls: true,
                wheel: false,
                startScale: 1.0,
                maxScale: 2.0,
                minScale: .4,
                scaleSpeed: 1.1,
                pinch: true
            },
            move: {
                scrollbars: {
                    horizontal: true,
                    vertical: true
                },
                drag: true,
                wheel: true
            },
            trashcan:true
        });
    let start_block = workspace.newBlock('start_block');
    start_block.initSvg();
    start_block.render();
    start_block.setDeletable(false);
    start_block.moveBy(200,10)

    let input = document.getElementById("file-input");
    input.onchange = e => {
        loadWorkspace(e.target.files[0]);
    }
}


/**
 * Generates list of Json Objects for stack of blocks in Blockly workspace.
 * */
function generateCode(){
    let blocks = workspace.getAllBlocks(true);
    let jsonPlan = [];
    let valid = true;

    for (let block of blocks) {
        // skip start block
        if (block.type === "start_block") {
            continue;
        }

        // subtract 1 for the dummy input holding the name of the block
        let parameterInputLength = block.inputList.length - 1;
        let jsonAction = {};
        jsonAction["params"] = [];

        if (block.parentBlock_ === null) {
            block.setWarningText("Delete unconnected blocks");
            valid = false;
        }
        // add connected, complete action blocks to plan if the workspace still looks valid so far
        else if (valid && parameterInputLength > 0) {
            // TODO get rid of magic numbers
            jsonAction["action_name"] = block.type.substring(0, block.type.length - 6);
            let paramNames = block.data.split(" ").filter(p => p !== "");
            for (let paramName of paramNames) {
                jsonAction["params"].push(block.getFieldValue(paramName));
            }

            jsonPlan.push(jsonAction);
            block.setWarningText(null);
        }
    }

    return valid ? jsonPlan : null;
}

/**
 * Deletes all blocks from Blockly workspace.
 **/
function clearWorkspace() {
    workspace.clear();
    let start_block = workspace.newBlock('start_block');
    start_block.initSvg();
    start_block.render();
    start_block.setDeletable(false);
    start_block.moveBy(200,10)
}

function showMsg(msg, msgType, blockType){

    let paraId = "statusmsg_"+blockType;
    let status_bar = document.getElementById(blockType);
    let status_para = document.getElementById(paraId);
    console.log(status_para);
    let para = document.createElement("p");
    para.className = msgType;
    para.id = paraId;
    paratext = document.createTextNode(msg);
    para.appendChild(paratext);

    if (status_para !== null){
        status_bar.removeChild(status_para);
    }
    status_bar.appendChild(para);
}

function clearMsg(blockType){
    // Remove the status message block from the screen
    let paraId = "statusmsg_"+blockType;
    let status_bar = document.getElementById(blockType);
    let status_para = document.getElementById(paraId);
    if (status_para !== null){
        status_bar.removeChild(status_para);
    }
}

/**
 * Saves the Blockly workspace.
 */
function saveWorkspace() {
    let fileName = "blockly-workspace.xml"
    let fileContent = Blockly.Xml.domToText(Blockly.Xml.workspaceToDom(workspace));
    let blob = new Blob([fileContent], {type: "text/xml"});
    saveData(blob, fileName);
}

function saveData(blob, fileName) {
    let a = document.createElement("a");
    document.body.appendChild(a);
    a.style.cssText = "display: none";
    let url = window.URL.createObjectURL(blob);
    a.href = url;
    a.download = fileName;
    a.click();
    window.URL.revokeObjectURL(url);
    a.remove();
}

/**
 * Loads a Blockly workspace.
 */
function loadWorkspace(file) {
    // TODO warn user of unsaved changes when they try to load?
    file.text().then(text => {
        Blockly.Xml.clearWorkspaceAndLoadFromXml(Blockly.Xml.textToDom(text), workspace);
    });
}
