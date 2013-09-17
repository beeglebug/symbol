var tester = new PDollarRecognizer();

var drawing = false;
var symbol = [];
var line = 0;
var gap = 1000;

document.addEventListener('mousedown', function(e) {
	drawing = true;
	//symbol = [];
	line++;
});

document.addEventListener('mousemove', function(e) {
	if(drawing) {
		symbol.push([e.offsetX, e.offsetY]);
	}
});

document.addEventListener('mouseup', function(e) {
	drawing = false;
	var result = tester.recognize([symbol]);
	console.log(result);
});
