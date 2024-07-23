// I made changes



var lastBadBlock = {
    "block":null,
    "colour":null,
    "set":false,
    "bad_blocks":[]
}

var blockColours = {}

function setBadBlock(index){
    let theBlocks = workspace.getAllBlocks(true);

    let block = theBlocks[index];
    if(block.getColour()!="#ec2020"){
        blockColours[block.id] = block.getColour()
    }
    block.setColour("#ec2020");    
}




// function unsetBadBlock(){

//     if (lastBadBlock['block'] !== null){
//         // if (workspace.getBlockById(lastBadBlock['block'].id) !== null){
//         //     lastBadBlock['block'].setColour(lastBadBlock['colour']);
//         //     lastBadBlock['block'] = null;
//         //     lastBadBlock['colour'] = null;
//         // }
//         if (workspace.getBlockById(lastBadBlock['block'].id) !== null){
//             let theBlocks = workspace.getAllBlocks(true);
//             for (let i = 0; i < theBlocks.length; i++) {
//                 if (theBlocks[i].getColour()=="#EC2020"){
//                     theBlocks[i].setColour(lastBadBlock["colour"])
//                 }
//             }
//             lastBadBlock['block'] = null;
//             lastBadBlock['colour'] = null;    
//         }




//     }

//     if(lastBadBlock['set']==true){
//         for (let i = 0; i < lastBadBlock['bad_blocks'].length; i++) {
//             if (workspace.getBlockById(lastBadBlock['bad_blocks'][i][0].id) !== null){
//                 lastBadBlock['bad_blocks'][i][0].setColour(lastBadBlock['bad_blocks'][i][1])
//             }
//           }

//           lastBadBlock['bad_blocks'] = []
//           lastBadBlock['set'] = false
//     }
// }


function clearCollapsibles(id, let_first = false) {
    const container = document.getElementById(id);
    // Determine the stopping condition based on let_first
    const stopCondition = let_first ? 2 : 0;

    // Keep removing the last child until the desired number of children remain
    while (container.children.length > stopCondition) {
      container.removeChild(container.lastChild);
    }
}



function clearCollapsiblesOfClass(className) {
    // Select all collapsible buttons with the specified class
    const buttons = document.querySelectorAll('.collapsible.' + className);
  
    buttons.forEach(button => {
      // Find the associated content div, which is expected to be the next sibling
      const contentDiv = button.nextElementSibling;
  
      // Remove the button
      if (button.parentNode) {
        button.parentNode.removeChild(button);
      }
  
      // Remove the associated content div if it exists and has the 'content' class
      if (contentDiv && contentDiv.classList.contains('content') && contentDiv.parentNode) {
        contentDiv.parentNode.removeChild(contentDiv);
      }
    });
  }
  


  function addCollapsible(index, heading, content, box_type,accordion_id) {
    const container = document.getElementById(accordion_id);
  
    // Create collapsible button
    const button = document.createElement('button');
    button.innerHTML = heading;
    // button.style.backgroundColor = color;
    button.classList.add('collapsible');
    button.classList.add(box_type);

    // Create content div
    const contentDiv = document.createElement('div');
    contentDiv.classList.add('content_new');

    contentDiv.innerHTML = `<p>${content}</p>`;


    // Append button and content div to container
    container.appendChild(button);
    container.appendChild(contentDiv);

    // Add click event listener to button
    button.addEventListener('click', function() {
      this.classList.toggle('active');
      const content = this.nextElementSibling;
      if (content.style.maxHeight) {
        content.style.maxHeight = null;
      } else {
        content.style.maxHeight = content.scrollHeight + 'px';
      }
    });

    button.addEventListener('mouseover', function() {
        highlightBlock(index);
      });
  }  

function createDropdowns(headings, contents,accordion_id) {
    const container = document.getElementById(accordion_id);
  
    headings.forEach((heading, index) => {
      const button = document.createElement('button');
      button.classList.add('collapsible');

        button.setAttribute("id", "state_" + index);
        button.setAttribute("index", index);
        button.innerHTML = heading;
        if (index % 2 === 0) {
            button.classList.add('dark-blue');
        } 
        else {
            button.classList.add('light-blue');
        }

      // Create content div
      const contentDiv = document.createElement('div');
      contentDiv.classList.add('content_new');
      const content = contents[index].replace(/\t/g, '&nbsp;&nbsp;&nbsp;&nbsp;').replace(/\n/g, '<br>') || 'No content available'; // Fallback content

      contentDiv.innerHTML = `<p>${content}</p>`;
  
      // Append button and content div to container
      container.appendChild(button);
      container.appendChild(contentDiv);
  
      // Add click event listener to button
      button.addEventListener('click', function() {
        this.classList.toggle('active');
        const content = this.nextElementSibling;
        if (content.style.maxHeight) {
          content.style.maxHeight = null;
        } else {
          content.style.maxHeight = content.scrollHeight + 'px';
        }
      });

      button.addEventListener('mouseover', function() {
        highlightBlock(index);
      });


    });
  }
  

function unsetBadBlock(){
    console.log("BLOCK Colours = ",blockColours)
    let theBlocks = workspace.getAllBlocks(true);
    for (let i = 0; i < theBlocks.length; i++) {
        if (theBlocks[i].getColour()=="#ec2020"){
            theBlocks[i].setColour(blockColours[theBlocks[i].id])
        }
    }
}

function call_tmp(){
    console.log("Calling TMP...")
    $.ajax({
        url: "start_TMP",
        type: "GET"
    }).done(function(data) { 
        console.log("TMP started again");
    })
}

// Terminate the execution by killing TMP process
function terminateExec(){

    enable_submit_button();
    enable_hint_button();
    flip_submit_button(true);

    let loader = document.getElementById("submission-loader");
    stop_loader_spinning("stop");

    $.ajax({
        url: 'terminate_tmp',
        type: 'GET'
    }).done(function(data){
        call_tmp();
        clearSimStatusBar();
    });


    // clearMsg('status'); // only clear the tmp status. Leave the explainer status intact.
    // clearMsg('explainer_status'); // only clear the tmp status. Leave the explainer status intact.
    // clearMsg('explainer_status2'); // only clear the tmp status. Leave the explainer status intact.
    // clearMsg('additional_explanation_status'); // only clear the tmp status. Leave the explainer status intact.

    // clearCollapsibles('accordion',true);
    // clearCollapsibles('accordion_status');

    // clearCollapsiblesOfClass("progress")
    clearSimStatusBar();


}

function submitGoal(goal){

    let goal_rand = 1;
    console.log("GOAL IN planSubmission.js = ",goal)
    
    $.ajax({
        url: "goal_submit",
        type: 'GET',
        data: {goal:JSON.stringify(goal)}
    }).done(function(data){
        console.log(data)

        document.getElementById('question').innerHTML = data['ans']
        return data
    });    


}


var loader_spin_count = 0;
function start_loader_spinning(line_no) {

    if (loader_spin_count == 0) {
        let loader = document.getElementById("submission-loader");
        loader.style.display = "block";
    }
    
    loader_spin_count += 1;
    console.log("Spinner start with E[" + line_no + "] current count is " + loader_spin_count)

}

function stop_loader_spinning(line_no){

    if (loader_spin_count > 0) {
        loader_spin_count -= 1;
    }

    console.log("Spinner  stop with E[" + line_no + "] current count is " + loader_spin_count)
    if (loader_spin_count == 0) {
        
        let loader = document.getElementById("submission-loader");
        loader.style.display = "none";
    }
}


function populateHint(hint){
    para = document.getElementById("hint_text");
    para.innerHTML = hint;
    return;
}

function populateSimStatusBar(heading){
    document.getElementById("simStatusFixed").innerHTML = heading;
    document.getElementById("simStatusFixed").style.backgroundColor = "#A9A9A9";
}

function clearSimStatusBar(){
    document.getElementById("simStatusFixed").innerHTML = "";
    document.getElementById("simStatusFixed").style.backgroundColor = "";
}

function populateFixedGoalBar(heading_main, heading_text,content,status){
    
    document.getElementById("fixed_goal_heading").innerHTML = "<b>" + heading_main + "</b>" + " " + heading_text;
    
    document.getElementById("fixed_goal_content").innerHTML = content.replaceAll(".","<br>");

    if (status=="bad") {
        document.getElementById("fixed_goal_heading").style.backgroundColor = "indianred";
        document.getElementById("fixed_goal_content").style.backgroundColor = "lightcoral";    
    }
    else if (status == "warn") {

        document.getElementById("fixed_goal_heading").style.backgroundColor = "#FFBC21";
        document.getElementById("fixed_goal_content").style.backgroundColor = "#FFE7AD";    
    }
    else {
        document.getElementById("fixed_goal_heading").style.backgroundColor = "#729C44";
        document.getElementById("fixed_goal_content").style.backgroundColor = "darkseagreen";    
    }

    if (content == "") {

        document.getElementById("fixed_goal_content").innerHTML = "";
        document.getElementById("fixed_goal_content").style.backgroundColor = ""; 
    }
}

function clearFixedGoalBar(){
    document.getElementById("fixed_goal_heading").innerHTML = "";    
    document.getElementById("fixed_goal_content").innerHTML = "";

    document.getElementById("fixed_goal_heading").style.backgroundColor = "";
    document.getElementById("fixed_goal_content").style.backgroundColor = "";    

}

var submitPlanHitOrder = 0;
var numSubmitStarts = 0;
var numSubmitEnds= 0;

var first_time = true;
var is_random_problem = false;
function submitPlan(plan,blob,get_hint=false,run_tmp=false){   

    if (!first_time && plan.length == 0 && !get_hint) {
        
        return;
    }

    first_time = false;
    let loader = document.getElementById("submission-loader");

    submitPlanHitOrder += 1;

    disable_submit_button();
    disable_hint_button();

    start_loader_spinning(1);

    clearMsg('status'); // only clear the tmp status. Leave the explainer status intact.
    clearMsg('explainer_status'); // only clear the tmp status. Leave the explainer status intact.
    clearMsg('explainer_status2'); // only clear the tmp status. Leave the explainer status intact.
    clearMsg('additional_explanation_status'); // only clear the tmp status. Leave the explainer status intact.
    
    console.log("get hints= ",get_hint)

    unsetBadBlock();
    
    let plan_all = "";
    for(let a = 0; a < plan.length; a++){
        plan_all += plan[a]["action_name"] + " " + plan[a]["params"].join(" ");
        if(a < plan.length-1)
            plan_all += ",";
    }

    numSubmitStarts += 1;
    $.ajax({
        url: "plan_submit",
        type: 'GET',
        data: { plan: plan_all, random : "HELLO FROM submitPlan javascript","get_hint":get_hint , submitPlanHitOrder:submitPlanHitOrder},
    }).done(function(data) {
        console.log("data.submitPlanHitOrder = ",data.submitPlanHitOrder)
        console.log("submitPlanHitOrder = ",submitPlanHitOrder)        
        numSubmitStarts--;

        if(data.helm_failure==='true'){
            console.log("CAUGH HELM ERROR!!!");
            stop_loader_spinning();
            populateFixedGoalBar("Goal not satisfied:", "Internal Error! Please Try Disconnecting and Connecting the blocks again", "", "bad");
            return;
        }

        if(data.submitPlanHitOrder < submitPlanHitOrder){
            console.log("These changes should not occur!!");
            stop_loader_spinning(1);
            return;
        }
        
        console.log("data returned from explainer= ",data)

        // let state_sequence = document.getElementById("state_sequence")
        // state_sequence.innerHTML = data['state_sequence'].replace(/\t/g, '&nbsp;&nbsp;&nbsp;&nbsp;').replace(/\n/g, '<br>');
        clearCollapsibles('accordion');
        clearCollapsibles('accordion_status');
        clearFixedGoalBar();

        createDropdowns(data['state_sequence'][0], data['state_sequence'][1],"accordion");

        
        if(plan_all==""){
            if(data['hint']!=''){
                hintMessage = 'You might want to try the action: '+  data['hint']
                clearMsg("hint_status")
                // showMsg( hintMessage, "pass","hint_status");
                populateHint(hintMessage);
            }
            stop_loader_spinning(1);
            enable_hint_button();
            return;
        }


        console.log("in planSubmission.js")
        console.log("data from submitPlan = ")
        console.log(data) 
        plan_all = data.execution_plan;
        let total_actions = data.explanation_map.total_actions;
        let executable_actions = data.explanation_map.exec_actions;


        // console.log(data['hint_gpt'])
        // console.log(data['explanation_gpt'])
        // console.log(data['state_sequence'])




        if (data.success === "True") {

            prob = document.getElementById('generate_random_goals').getAttribute("problem") 
            domain = document.getElementById('generate_random_goals').getAttribute("domain") 
            
            file_name = "/root/git/JEDAI/successful_plans/"+domain.split("/").slice(-1)[0]+"_"+prob+".xml"

            $.ajax({
                url: "save_plan/",
                type: 'GET',
                data: { blob: blob,file_name:file_name},
            }).done(function(data) {
                console.log(data)
            })

 
            let successCallMsg = "VALID: Plan correct! All the actions will be executed"

            if (!is_random_problem) {

                populateFixedGoalBar("Goal Satisfied:", "The plan will reach the goal. Press the 'Execute on Robot' button to execute it on the robot.", "", "good");
            }
            else {

                populateFixedGoalBar("Goal Satisfied:", "The plan will reach the goal. Please generate a new problem using the 'Create Curriculum Problem' button.", "", "good");
            }

            explainer_success = "pass";
        }
        else {
            // Do all the colour checking ops only if the explainer returns a failed precondition
            if ("failed_precondition" in data.explanation_map){
                if ("badStep" in data.explanation_map){

                    // let badStep = parseInt(data.explanation_map.badStep);
                    let badStep = parseInt(data.explanation_map.bad_action_step_helm);

                    setBadBlock(badStep+1);                    

                }

            }

            if ("failed_precondition" in data.explanation_map_non_helm){
                if ("badStep" in data.explanation_map_non_helm){

                    let badStep = parseInt(data.explanation_map_non_helm.bad_action_step_non_helm);
                    setBadBlock(badStep+1);                    
    
    
                    if(data['get_hint']){
                        populateHint("Please fix errors and try again")
                    }
    
                }
                        
            }

            let code1 = data.explanation_map_non_helm.err_code;
            let code2 = data.explanation_map.err_code;
 
            if(code1==='GOAL ERROR' && code2==='GOAL ERROR'){
                let failure_cause_1 = data.explanation_map_non_helm["failure_cause"]
                let failure_cause_2 = data.explanation_map["failure_cause"];

                if(failure_cause_1!==failure_cause_2){
                    failure_cause_1 += failure_cause_2;
                }
                populateFixedGoalBar("Goal not satisfied:", "Valid plan but does not satisfy goal", failure_cause_1, "warn");
            }
            else if(code1!=='GOAL ERROR' && code2!=='GOAL ERROR'){
                populateFixedGoalBar("Goal not satisfied:", "Invalid plan", "", "bad");

                let badStep1 = parseInt(data.explanation_map_non_helm.bad_action_step_non_helm)+1;
                let badStep2 = parseInt(data.explanation_map.bad_action_step_helm)+1;

                let failure_cause_1 = data.explanation_map_non_helm["failure_cause"]
                let failure_cause_2 = data.explanation_map["failure_cause"];

                let failureCallMsg1 = "<b>TLDR: </b>" + failure_cause_1 + "<br> <br>" + data['explanation_non_helm_gpt'];
                let failureCallMsg2 = "<b>TLDR: </b>" + failure_cause_2 + "<br> <br>" + data['explanation_helm_gpt'];


                if(failure_cause_1===failure_cause_2){
                    addCollapsible(badStep1, "Invalid Action " + badStep1, failureCallMsg1, 'bad_action',"accordion_status")                            
                }
                else if(badStep1 != badStep2){
                    console.log("ADDING 2 for the different actions");
                    addCollapsible(badStep1, "Invalid Action " + badStep1, failureCallMsg1, 'bad_action',"accordion_status")    
                    addCollapsible(badStep2, "Invalid Action " + badStep2, failureCallMsg2, 'bad_action',"accordion_status")    
                }
                else{
                    addCollapsible(badStep1, "Invalid Action " + badStep1, failureCallMsg1 + "<br>" + failureCallMsg2, 'bad_action',"accordion_status")
                }
            }
            else{
                console.log("Different codes");
                let badStep1 = parseInt(data.explanation_map_non_helm.bad_action_step_non_helm)+1;
                let badStep2 = parseInt(data.explanation_map.bad_action_step_helm)+1;

                let failure_cause_1 = data.explanation_map_non_helm["failure_cause"]
                let failure_cause_2 = data.explanation_map["failure_cause"];

                let failureCallMsg1 = "<b>TLDR: </b>" + failure_cause_1 + "<br> <br>" + data['explanation_non_helm_gpt'];
                let failureCallMsg2 = "<b>TLDR: </b>" + failure_cause_2 + "<br> <br>" + data['explanation_helm_gpt'];

                if(code1==="GOAL ERROR"){
                    populateFixedGoalBar("Goal not satisfied:","Invalid plan", "", "bad");
                    addCollapsible(badStep2, "Invalid Action "+badStep2, failureCallMsg2, 'bad_action',"accordion_status");
                }
                else{
                    populateFixedGoalBar("Goal not satisfied:", "Invalid plan", "", "bad");
                    addCollapsible(badStep1, "Invalid Action "+badStep1, failureCallMsg1, 'bad_action',"accordion_status")    
                }
            }

            additional_explantion = ""

            if(data['hint_gpt']!=""){
                additional_explantion += data['hint_gpt'];
            }

            // if(data['explanation_gpt']!="No errors"){
            //     additional_explantion += data['explanation_gpt'];
            // }

            // if(additional_explantion!=""){
            //     // showMsg(additional_explantion , "pass","additional_explanation_status");
            //     addCollapsible("Additional Information", additional_explantion, 'additional_info',"accordion_status")

            // }
              
            explainer_success = "fail";
        }
        

        if(get_hint && data['err_code_helm']=="GOAL ERROR"&& data['err_code_non_helm']=="GOAL ERROR"){
            hintMessage = 'You might want to try the action: '+  data['hint']
            clearMsg("hint_status")
            // showMsg( hintMessage, "pass","hint_status");
            populateHint(hintMessage);
        }

        // Only call TMP if the plan is not empty
        // An empty plan submission would never reach here normally but a pruned plan may be empty even if there are action blocks set up in the workspace
        // if (plan_all != '' && !(get_hint) && data.success === "True" && run_tmp){

        stop_loader_spinning(1);

        const no_error = (data['err_code_helm']=="GOAL ERROR" && data['err_code_non_helm']=="GOAL ERROR") || data.success === "True";

        if (plan_all != '' && !(get_hint) && no_error && run_tmp){
            
            start_loader_spinning(2);
            // if (plan_all != '' && !(get_hint) && run_tmp){
            console.log("CALLING TMP because no get_hints!")
            // let generalTMPFillerMsg = "INFO: Calling TMP to convert plan to robot movements. You may see some movement in the 3D environment window as computations take place.";
            let generalTMPFillerMsg = "<b>Sim Status:</b> Computing Motion Plan for Robot";
            // showMsg(generalTMPFillerMsg, "inprogress", "status")    
            // addCollapsible("In Progress", generalTMPFillerMsg, "progress","accordion_status")
            populateSimStatusBar(generalTMPFillerMsg);


            enable_submit_button();
            flip_submit_button(false);

            $.ajax({
                
                url: "call_TMP",
                type: 'GET',
                data: { plan: plan_all }
                }).done(function(data) {
                console.log("TMP_______CALLED");

                if (data.success === "True") {
                    // let finishedComputingAlert = "TMP finished computing. Close this alert to allow your plan to begin executing, then watch the 3D environment window!";
                    let finishedComputingAlert = "Plan Generated ! Close this alert to allow your plan to begin executing, then watch the 3D environment window!";
                    let finishedComputingMsg = "<b>Sim Status:</b> Executing Plan"
                    // showMsg(finishedComputingMsg, "inprogress","status");
                    // addCollapsible("In Progress",finishedComputingMsg,"progress","accordion_status")
                    populateSimStatusBar(finishedComputingMsg);
                    $.ajax({
                        url: 'run_plan',
                        type: 'GET',
                        beforeSend: alert(finishedComputingAlert)
                    }).done(function(data) {
                        let tmpCompletionAlert = "Executed "+executable_actions+" of "+total_actions+" action(s) successfully! Close this alert to reset the 3D environment.";
                        let tmpCompletionMsg = "<b>Sim Status:</b> Plan execution complete!"
                        // showMsg(tmpCompletionMsg, "inprogress","status");
                        // addCollapsible("In Progress",tmpCompletionMsg,"progress","accordion_status")
                        populateSimStatusBar(tmpCompletionMsg);
                        $.ajax({
                            url: 'kill_tmp',
                            type: 'GET',
                            beforeSend: alert(tmpCompletionAlert)
                        }).done(function(data){
                            call_tmp();
                        });
                    });
                }
                else {
                    let tmpErrorAlert = "If the 3D environment window doesn't open again, you may need to restart JEDAI.";
                    // showMsg(tmpErrorAlert, "fail");
                    populateSimStatusBar("<b>Sim Status:</b> Sim ERROR! Need to restart system.");
                    alert(tmpErrorAlert);
                    call_tmp();
                }
                stop_loader_spinning(2);
                flip_submit_button(true);
                enable_hint_button();
            });

        } else if (no_error) {

            // Only enable the submit button if the goal is reached.
            // This is to match JEDAI for the user study.
            //
            // TODO: Enable for partial execution.
            if (data.success === "True") {

                enable_submit_button();
            }
            enable_hint_button();

        }
        else {
            enable_hint_button();
        }
    });
}
