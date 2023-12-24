submitted = true;

function administerCompetition(pendulum) {
	if (typeof competition === 'undefined') {
		competition = {state: "not_satisfied", satisfiedTime: null, setpointsReached: 0, startTime: Date.now(), satisfiedCount: 0, totalCount: 0}
	}

	// if (typeof renderer_plot === "undefined") {
	// 	updateScene()
	// }



	checkSetpoint(pendulum)
}

function satisfiedTest(pendulum) {
	angle_threshold = 1e-2;
	angle_test = Math.abs(pendulum.angle - pendulum.desired) < angle_threshold;

	angle_dot_threshold = 1e-2;
	angle_dot_test = Math.abs(pendulum.angle_dot) < angle_dot_threshold;

	return angle_test && angle_dot_test;
}


function checkSetpoint(pendulum) {
	textbar.innerHTML += "<br><br> Competition" +
						 "<br>" +
            			 " setpointsReached = " + competition.setpointsReached +
            			 "<br>" +
            			 " setpointsAverage = " + (60 * competition.setpointsReached / (1e-3 * (Date.now() - competition.startTime))).toFixed(2)

	//want to also count the number of frames satisfying the test
	if (satisfiedTest(pendulum)) {
		if (competition.state === "not_satisfied") {
			competition.state = "satisfied";
			competition.satisfiedTime = Date.now();
			competition.satisfiedCount += 1;
			return
		}
		else {
			if ((Date.now() - competition.satisfiedTime) % 1000 == 0 && (Date.now() - competition.satisfiedTime) <= 5000) {
				competition.satisfiedCount += 1;
			}
			if ((Date.now() - competition.satisfiedTime) > 5000) {
				competition.setpointsReached += 1;
				competition.state = "not_satisfied";
				competition.totalCount += competition.satisfiedCount;
				competition.satisfiedTime = null;
				competition.satisfiedCount = 0;
				pendulum.desired = ((Math.random() * 2) - 1) * 2 * Math.PI;

			}
		}
	}

	if((Date.now() - competition.startTime) > 60000 * competition_time) {
		timeup = true;
		in_competition = false;
		console.log("Competition ended");
	}
}
