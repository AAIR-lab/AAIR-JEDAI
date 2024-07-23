// const fs = require('fs')


var workspace;
var workspace2;
var latest_event;





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
                // maxScale: 4.0,
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

    // workspace2 = Blockly.inject('blocklyDiv2',
    //     {
    //         toolbox: document.getElementById('toolbox2'),
    //         zoom: {
    //             controls: true,
    //             wheel: false,
    //             startScale: 1.0,
    //             maxScale: 2.0,
    //             minScale: .4,
    //             scaleSpeed: 1.1,
    //             pinch: true
    //         },
    //         move: {
    //             scrollbars: {
    //                 horizontal: true,
    //                 vertical: true
    //             },
    //             drag: true,
    //             wheel: true
    //         },
    //         trashcan:true
    // });
    // let start_block2 = workspace2.newBlock('start_block');
    // start_block2.initSvg();
    // start_block2.render();
    // start_block2.setDeletable(false);
    // start_block2.moveBy(500,500)
    
    console.log("INITIALIZE BLOCKAS!!!!")


    let input = document.getElementById("file-input");
    input.onchange = e => {
        loadWorkspace(e.target.files[0]);
    }
}


/**
 * Generates list of Json Objects for stack of blocks in Blockly workspace.
 * */
function generateCustomGoal(){
    let blocks = workspace2.getAllBlocks(true);

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

        // if (block.parentBlock_ === null) {
        //     block.setWarningText("Delete unconnected blocks");
        //     valid = false;
        // }
        // add connected, complete action blocks to plan if the workspace still looks valid so far
        if (valid && parameterInputLength > 0) {
            // TODO get rid of magic numbers
            jsonAction["pred_name"] = block.type.substring(0, block.type.length - 6);
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


function highlightBlock(index) {

    blocks = workspace.getAllBlocks();
    if (isNaN(index) || index >= blocks.length) {
        
        return;
    }
    else {

        blocks[index].select();
    }
}

function generateCode(){
    let blocks = workspace.getAllBlocks(true);
    let jsonPlan = [];

    // jsonPlan['fileContent'] = Blockly.Xml.domToText(Blockly.Xml.workspaceToDom(workspace));
    let valid = true;

    for (let block of blocks) {
        // skip start block
        if (block.type === "start_block") {
            continue;
        }

        if(block.type=='loops_block'||blocks.type=='goals_block'){
            stochastic_plan(blocks);
            return "Run the Stochastic Plan";
        }

        // subtract 1 for the dummy input holding the name of the block
        let parameterInputLength = block.inputList.length - 1;
        let jsonAction = {};
        jsonAction["params"] = [];

        if (block.parentBlock_ === null) {
            console.log("UNCONNECTED BLOCK!!")
            block.setWarningText("Need to connect this block");
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


    

    return valid ? [jsonPlan,Blockly.Xml.domToText(Blockly.Xml.workspaceToDom(workspace))] : null;
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
    para.innerHTML = msg.replace(/\t/g, '&nbsp;&nbsp;&nbsp;&nbsp;').replace(/\n/g, '<br>');
    para.className = msgType;
    para.id = paraId;
    // paratext = document.createTextNode(msg.replace(/\t/g, '&nbsp;&nbsp;&nbsp;&nbsp;').replace(/\n/g, '<br>'));
    // para.appendChild(paratext);

    // let btn = document.createElement('button');
    // btn.innerHTML = "x";
    // btn.onclick = clearMsg(blockType);
    // para.appendChild(btn)

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

    console.log("paraId = ",paraId);

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

// function saveWorkspaceSuccess(fileName){
 
//     let fileContent = Blockly.Xml.domToText(Blockly.Xml.workspaceToDom(workspace));
//     fileName = '/root/git/JEDAI/successful_plans/'+fileName
//     let blob = new Blob([fileContent], {type: "text/xml"});
//     // fs.writeFile(fileName, blob, (err) => {
          
//     //     // In case of a error throw err.
//     //     if (err) throw err;
//     // })
// }


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
};





