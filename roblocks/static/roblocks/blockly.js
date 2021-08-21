function submitPlan(plan)
{
    let loader = document.getElementById("submission-loader");
    loader.style.display = "block";
    console.log("Submitted plan: " + plan);
    let plan_all = "";
    for(let a = 0; a < plan.length; a++){
        plan_all += plan[a]["action_name"] + " " + plan[a]["params"].join(" ");
        if(a < plan.length-1)
            plan_all += ",";
    }
    console.log("____________",plan_all);

    var call_tmp = function(){ 
        console.log("calling TMp again...")
        $.ajax({
            url : "start_TMP",
            type: "GET"
        }).done(function(data) { 
            console.log("TMP started again");
        })
    }

    $.ajax({
        url : "plan_submit",
        type : 'GET',
        data :
            {
                plan: plan_all,
            },

    }).done(function(data) {
        if (data.success === "True") {
            alert("Calling TMP to convert plan to robot movements. You may see some movement in the 3D environment window as computations take place.")
            $.ajax({
                url : "call_TMP",
                type : 'GET',
                data : {
                        plan: plan_all,
                }
            }).done(function(data) {
                console.log("TMP_______CALLED");
                if (data.success === "True") {
                    alert("TMP finished computing the solution. Open the 3D environment window to see the execution!");
                    $.ajax({
                        url: 'run_plan',
                        type: 'GET'
                    }).done(function(data) {
                        alert("TMP executed plan with success");
                        $.ajax({
                            url: 'kill_tmp',
                            type: 'GET'
                        }).done( function(data){
                            call_tmp();
                        }
                        )
                    })
                }
                else {
                    alert("TMP reported failure. If the 3D environment window doesn't open again, you may need to restart JEDAI."); // TODO: Verify these strings.
                    call_tmp();
                }
                loader.style.display = "none";
            })

        }
        else {
            alert(data.explanation_map["failure_cause"]);
            loader.style.display = "none";
        }
    });
}
