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


/**
 * a 2D Point
 * @constructor
 * @param {Number} x horizontal position
 * @param {Number} y vertical position
 * @param {Number} id path id
 */
var Point = function(x, y, id) {

	this.x = x;
	this.y = y;
	this.id = id;
};


/**
 * Calculate the distance to another Point
 * @param {Point} point another Point
 * @return {Number} the euclidean distance between the two points
 */
Point.prototype.distanceTo = function(point) {

	var dx = point.x - this.x;
	var dy = point.y - this.y;

	return Math.sqrt(dx * dx + dy * dy);
};


/**
 * A collection of Points representing a symbol
 * @constructor
 * @param {String} name a name for the symbol
 * @param {Array} pointsList the Points which make up the symbol
 */
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
};


/**
 * evenly space the points in the cloud
 * @return {Array} the new points
 */
PointCloud.prototype.resample = function() {

	var points = this.points;
	var I = this.pathLength() / (this.resolution - 1); // interval length
	var D = 0.0;
	var newpoints = new Array(points[0]);
	for (var i = 1; i < points.length; i++) {
		if (points[i].id == points[i-1].id) {
			var d = points[i - 1].distanceTo(points[i]);
			if ((D + d) >= I) {
				var qx = points[i - 1].x + ((I - D) / d) * (points[i].x - points[i - 1].x);
				var qy = points[i - 1].y + ((I - D) / d) * (points[i].y - points[i - 1].y);
				var q = new Point(qx, qy, points[i].id);
				newpoints[newpoints.length] = q; // append new point 'q'
				points.splice(i, 0, q); // insert 'q' at position i in points s.t. 'q' will be the next i
				D = 0.0;
			} else {
				D += d;
			}
		}
	}
	if (newpoints.length == this.resolution - 1) { // sometimes we fall a rounding-error short of adding the last point, so add it if so
		newpoints[newpoints.length] = new Point(points[points.length - 1].x, points[points.length - 1].y, points[points.length - 1].id);
	}

	return newpoints;
};


/**
 * get the total length of all paths in the point cloud
 * @return {Number} the total length
 */
PointCloud.prototype.pathLength = function() {

	var length = 0;

	for (var i = 1; i < this.points.length; i++) {
		if (this.points[i].id == this.points[i-1].id) {
			length += this.points[i - 1].distanceTo(this.points[i]);
		}
	}

	return length;
};


/**
 * scale the point cloud to a uniform size for comparison purposes
 * @return {Array} the new points
 */
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
};


/**
 * translate all points in the point cloud to an origin for comparison purposes
 * @return {Array} the new points
 */
PointCloud.prototype.translateTo = function(origin) {

	var centroid = this.centroid();
	var newpoints = new Array();

	for (var i = 0; i < this.points.length; i++) {
		newpoints[newpoints.length] = new Point(
			this.points[i].x + origin.x - centroid.x,
			this.points[i].y + origin.y - centroid.y,
			this.points[i].id
		);
	}

	return newpoints;
};


/**
 * calculate the center of the cloud
 * @return {Point} a new Point representing the exact center of the cloud
 */
PointCloud.prototype.centroid = function(points) {

	var centroid = new Point(0,0,0);

	for (var i = 0; i < this.points.length; i++) {
		centroid.x += this.points[i].x;
		centroid.y += this.points[i].y;
	}

	centroid.x /= this.points.length;
	centroid.y /= this.points.length;

	return centroid;
};


/**
 * SymbolRecognizer
 * @constructor
 */
var SymbolRecognizer = function() {

	this.pointClouds = [];
	this.listeners = {};

	this.registerEvents(document);
};


/**
 * [registerEvents description]
 * @param  {[type]} target [description]
 * @return {[type]}        [description]
 */
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


/**
 * [on description]
 * @param  {[type]}   name     [description]
 * @param  {Function} callback [description]
 * @return {[type]}            [description]
 */
SymbolRecognizer.prototype.on = function(name, callback) {

	if(!this.listeners[name]) {
		this.listeners[name] = [];
	}

	this.listeners[name].push(callback);
};


/**
 * [recognize description]
 * @param  {[type]} pointsList [description]
 * @return {[type]}            [description]
 */
SymbolRecognizer.prototype.recognize = function(pointsList) {

	// If not enough points are passed in make sure it doesn't break.
	if ( pointsList[0].length <= 1 ) { return; }

	var cloud = new PointCloud("user", pointsList);

	var b = +Infinity;
	var u = -1;
	for (var i = 0; i < this.pointClouds.length; i++) { // for each point-cloud template
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


/**
 * Add a new symbol to the recognizer
 * @param {String} name a name for the symbol
 * @param {Array} pointList the points which make up the symbol
 */
SymbolRecognizer.prototype.add = function(name, pointList) {

	this.pointClouds.push( new PointCloud(name, pointList) );

};


/**
 * [greedyCloudMatch description]
 * @param  {[type]} cloud [description]
 * @param  {[type]} P     [description]
 * @return {[type]}       [description]
 */
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
};


/**
 * [cloudDistance description]
 * @param  {[type]} pts1  [description]
 * @param  {[type]} pts2  [description]
 * @param  {[type]} start [description]
 * @return {[type]}       [description]
 */
SymbolRecognizer.prototype.cloudDistance = function(pts1, pts2, start) {

	var matched = new Array(pts1.length); // pts1.length == pts2.length
	for (var k = 0; k < pts1.length; k++) {	matched[k] = false;	}
	var sum = 0;
	var i = start;
	do {
		var index = -1;
		var min = +Infinity;
		for (var j = 0; j < matched.length; j++) {
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
};


/**
 * Set configuration options for the recognizer
 * @param  {[type]} options [description]
 * @return {[type]}         [description]
 */
SymbolRecognizer.prototype.config = function(options) {

	// @todo

};

// export global
window.Symbol = new SymbolRecognizer();

})(window);