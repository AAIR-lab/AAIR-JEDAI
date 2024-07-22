
function set_button_disabled(button_id, value) {

    button = document.getElementById(button_id)
    button.disabled = value
}

function enable_submit_button() {

    set_button_disabled("sendPlanBtn", false)
}

function disable_submit_button() {

    set_button_disabled("sendPlanBtn", true)
}

function enable_hint_button() {

    set_button_disabled("get_a_hint", false)
}

function disable_hint_button() {

    set_button_disabled("get_a_hint", true)
}

function flip_submit_button(execute_on_robot) {

    let submitBtn = document.getElementById("sendPlanBtn");
    if (execute_on_robot) {

        submitBtn.className = "btn btn-success"
        submitBtn.textContent = "Execute on Robot"
        submitBtn.onclick = function() {
            sendPlan(get_hint=false, run_tmp=true)
        }
    }
    else {

        submitBtn.className = "btn btn-danger"
        submitBtn.textContent = "Stop Execution"
        submitBtn.onclick = terminateExec
    }   
}