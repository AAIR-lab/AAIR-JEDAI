

function getRandGoal(domain,problem,csrf_token){
    $.ajax({
        url: "get_random/",
        type: 'POST',
        data: { domain: domain, problem : problem , csrfmiddlewaretoken: csrf_token
    }
    }).done(function(data) {
        // console.log("data from get_random = ",data)
        return data
    })
}