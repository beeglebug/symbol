
function Point(x, y, id) {
	this.x = x;
	this.y = y;
	this.id = id;
}

function PointCloud(name, pointsList) {

	this.resolution = 32;
	this.name = name;

	this.points = [];

	for(var r = 0; r < pointsList.length; r++) {
		for(var i = 0; i < pointsList[r].length; i++) {
			this.points.push(new Point( pointsList[r][i][0], pointsList[r][i][1], r ));
		}
	}

	this.points = this.resample();
	this.points = this.scale();
	this.points = this.translateTo(new Point(0,0,0));
}

PointCloud.prototype.resample = function() {
	var points = this.points;
	var I = this.pathLength() / (this.resolution - 1); // interval length
	var D = 0.0;
	var newpoints = new Array(points[0]);
	for (var i = 1; i < points.length; i++)
	{
		if (points[i].id == points[i-1].id)
		{
			var d = Distance(points[i - 1], points[i]);
			if ((D + d) >= I)
			{
				var qx = points[i - 1].x + ((I - D) / d) * (points[i].x - points[i - 1].x);
				var qy = points[i - 1].y + ((I - D) / d) * (points[i].y - points[i - 1].y);
				var q = new Point(qx, qy, points[i].id);
				newpoints[newpoints.length] = q; // append new point 'q'
				points.splice(i, 0, q); // insert 'q' at position i in points s.t. 'q' will be the next i
				D = 0.0;
			}
			else D += d;
		}
	}
	if (newpoints.length == this.resolution - 1) // sometimes we fall a rounding-error short of adding the last point, so add it if so
		newpoints[newpoints.length] = new Point(points[points.length - 1].x, points[points.length - 1].y, points[points.length - 1].id);
	return newpoints;
}

PointCloud.prototype.pathLength = function() {
	var d = 0.0;
	for (var i = 1; i < this.points.length; i++)
	{
		if (this.points[i].id == this.points[i-1].id)
			d += Distance(this.points[i - 1], this.points[i]);
	}
	return d;
}

PointCloud.prototype.scale = function() {
	var points = this.points;
	var minX = +Infinity, maxX = -Infinity, minY = +Infinity, maxY = -Infinity;
	for (var i = 0; i < points.length; i++) {
		minX = Math.min(minX, points[i].x);
		minY = Math.min(minY, points[i].y);
		maxX = Math.max(maxX, points[i].x);
		maxY = Math.max(maxY, points[i].y);
	}
	var size = Math.max(maxX - minX, maxY - minY);
	var newpoints = new Array();
	for (var i = 0; i < points.length; i++) {
		var qx = (points[i].x - minX) / size;
		var qy = (points[i].y - minY) / size;
		newpoints[newpoints.length] = new Point(qx, qy, points[i].id);
	}
	return newpoints;
}

PointCloud.prototype.translateTo = function(pt) {
	var points = this.points;
	var c = this.centroid();
	var newpoints = new Array();
	for (var i = 0; i < points.length; i++) {
		var qx = points[i].x + pt.x - c.x;
		var qy = points[i].y + pt.y - c.y;
		newpoints[newpoints.length] = new Point(qx, qy, points[i].id);
	}
	return newpoints;
}

PointCloud.prototype.centroid = function(points) {
	var points = this.points;
	var x = 0.0, y = 0.0;
	for (var i = 0; i < points.length; i++) {
		x += points[i].x;
		y += points[i].y;
	}
	x /= points.length;
	y /= points.length;
	return new Point(x, y, 0);
}

function PDollarRecognizer() {

	this.pointClouds = [];

	this.pointClouds.push( new PointCloud("N", [
		[[177,92],[177,2]],
		[[182,1],[246,95]],
		[[247,87],[247,1]]
	]));

	this.pointClouds.push( new PointCloud("line", [
		[[12,347],[119,347]]
	]));

	this.recognize = function(pointsList) {

		var cloud = new PointCloud("user", pointsList);

		var b = +Infinity;
		var u = -1;
		for (var i = 0; i < this.pointClouds.length; i++) // for each point-cloud template
		{
			var d = this.greedyCloudMatch(cloud, this.pointClouds[i]);
			if (d < b) {
				b = d; // best (least) distance
				u = i; // point-cloud
			}
		}
		return (u == -1) ? 'new result' : [ this.pointClouds[u].name, Math.max((b - 2.0) / -2.0, 0.0) ];
	};
}


PDollarRecognizer.prototype.greedyCloudMatch = function(cloud, P) {
	var points = cloud.points;
	var e = 0.50;
	var step = Math.floor(Math.pow(points.length, 1 - e));
	var min = +Infinity;
	for (var i = 0; i < points.length; i += step) {
		var d1 = this.cloudDistance(points, P.points, i);
		var d2 = this.cloudDistance(P.points, points, i);
		min = Math.min(min, Math.min(d1, d2)); // min3
	}
	return min;
}

PDollarRecognizer.prototype.cloudDistance = function(pts1, pts2, start)
{
	var matched = new Array(pts1.length); // pts1.length == pts2.length
	for (var k = 0; k < pts1.length; k++)
		matched[k] = false;
	var sum = 0;
	var i = start;
	do
	{
		var index = -1;
		var min = +Infinity;
		for (var j = 0; j < matched.length; j++)
		{
			if (!matched[j]) {
				var d = Distance(pts1[i], pts2[j]);
				if (d < min) {
					min = d;
					index = j;
				}
			}
		}
		matched[index] = true;
		var weight = 1 - ((i - start + pts1.length) % pts1.length) / pts1.length;
		sum += weight * min;
		i = (i + 1) % pts1.length;
	} while (i != start);
	return sum;
}

function Distance(p1, p2) // Euclidean distance between two points
{
	var dx = p2.x - p1.x;
	var dy = p2.y - p1.y;
	return Math.sqrt(dx * dx + dy * dy);
}