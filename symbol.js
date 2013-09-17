/**
 * The MIT License (MIT)
 * Copyright (c) 2013 Stuart Lee
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Original P$ algorithm from: http://depts.washington.edu/aimgroup/proj/dollar/pdollar.html
 */
;(function(window, undefined) {

var Point = function(x, y, id) {
	this.x = x;
	this.y = y;
	this.id = id;
}

Point.prototype.distanceTo = function(p2)
{
	var dx = p2.x - this.x;
	var dy = p2.y - this.y;
	return Math.sqrt(dx * dx + dy * dy);
}

var PointCloud = function(name, pointsList) {

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
			var d = points[i - 1].distanceTo(points[i]);
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
			d += this.points[i - 1].distanceTo(this.points[i]);
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

var SymbolRecognizer = function() {

	this.pointClouds = [];

	this.listeners = {};

	this.registerEvents(document);

}

SymbolRecognizer.prototype.registerEvents = function(target) {

	var drawing = false;
	var symbol = [];
	var line = 0;
	var self = this;

	target.addEventListener('mousedown', function(e) {
		drawing = true;
		symbol = [];
		line++;
	});

	target.addEventListener('mousemove', function(e) {
		if(drawing) {
			symbol.push([e.offsetX, e.offsetY]);
		}
	});

	target.addEventListener('mouseup', function(e) {
		drawing = false;
		var result = self.recognize([symbol]);
	});

};

SymbolRecognizer.prototype.on = function(name, callback) {

	if(!this.listeners[name]) {
		this.listeners[name] = [];
	}

	this.listeners[name].push(callback);

};

SymbolRecognizer.prototype.recognize = function(pointsList) {

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
	var match = this.pointClouds[u].name;
	var val = Math.max((b - 2.0) / -2.0, 0.0);

	if(val <= 0) { return false; }

	if(this.listeners[match]) {
		for(var i = 0; i < this.listeners[match].length; i++) {
			this.listeners[match][i](val);
		}
	}
};


SymbolRecognizer.prototype.add = function(name, pointList) {

	this.pointClouds.push( new PointCloud(name, pointList) );

};


SymbolRecognizer.prototype.greedyCloudMatch = function(cloud, P) {
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

SymbolRecognizer.prototype.cloudDistance = function(pts1, pts2, start)
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
				var d = pts1[i].distanceTo(pts2[j]);
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


	window.Symbol = new SymbolRecognizer();

})(window);