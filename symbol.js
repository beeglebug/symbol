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

Point.prototype.clone = function() {
	return new Point(this.x, this.y, this.id);
};

Point.prototype.add = function(point) {
	this.x += point.x;
	this.y += point.y;
	return this;
};

Point.prototype.subtract = function(point) {
	this.x -= point.x;
	this.y -= point.y;
	return this;
};

Point.prototype.multiply = function(scalar) {
	this.x *= scalar;
	this.y *= scalar;
	return this;
};

Point.prototype.divide = function(scalar) {
	this.x /= scalar;
	this.y /= scalar;
	return this;
};

Point.prototype._add = function(point) { return this.clone().add(point); }
Point.prototype._subtract = function(point) { return this.clone().subtract(point); }
Point.prototype._multiply = function(scalar) { return this.clone().multiply(scalar); }
Point.prototype._divide = function(scalar) { return this.clone().divide(scalar); }

/**
 * A collection of Points representing a symbol
 * @constructor
 * @param {String} name a name for the symbol
 * @param {Array} pointsList the Points which make up the symbol
 */
var PointCloud = function(name, pointsList) {

	this.resolution = 32;
	this.name = name;

	// keep a copy
	this.originalPoints = pointsList.slice(0);
	this.points = [];

	for(var path = 0; path < pointsList.length; path++) {
		for(var point = 0; point < pointsList[path].length; point++) {
			this.points.push( new Point( pointsList[path][point][0], pointsList[path][point][1], path ) );
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
	var resampled = new Array(points[0]);
	for (var i = 1; i < points.length; i++) {
		if (points[i].id == points[i-1].id) {
			var d = points[i - 1].distanceTo(points[i]);
			if ((D + d) >= I) {
				var qx = points[i - 1].x + ((I - D) / d) * (points[i].x - points[i - 1].x);
				var qy = points[i - 1].y + ((I - D) / d) * (points[i].y - points[i - 1].y);
				var q = new Point(qx, qy, points[i].id);
				resampled[resampled.length] = q; // append new point 'q'
				points.splice(i, 0, q); // insert 'q' at position i in points s.t. 'q' will be the next i
				D = 0.0;
			} else {
				D += d;
			}
		}
	}
	if (resampled.length == this.resolution - 1) { // sometimes we fall a rounding-error short of adding the last point, so add it if so
		resampled[resampled.length] = new Point(points[points.length - 1].x, points[points.length - 1].y, points[points.length - 1].id);
	}

	return resampled;
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

	var scaled = [];

	var min = new Point(+Infinity, +Infinity, 0);
	var max = new Point(-Infinity, -Infinity, 0);

	// calculate min and max bounds
	for (var i = 0; i < this.points.length; i++) {
		min.x = Math.min(min.x, this.points[i].x);
		min.y = Math.min(min.y, this.points[i].y);
		max.x = Math.max(max.x, this.points[i].x);
		max.y = Math.max(max.y, this.points[i].y);
	}

	var size = Math.max(max.x - min.x, max.y - min.y);

	for (var i = 0; i < this.points.length; i++) {
		scaled.push( this.points[i]._subtract(min).divide(size) );
	}

	return scaled;
};


/**
 * translate all points in the point cloud to an origin for comparison purposes
 * @return {Array} the new points
 */
PointCloud.prototype.translateTo = function(origin) {

	var translated = [],
		centroid = this.centroid(),
		offset = origin._subtract(centroid);

	for (var i = 0, len = this.points.length; i < len; i++) {
		translated.push( this.points[i]._add(offset) );
	}

	return translated;
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
 * total distance to another point cloud
 * @param  {[type]} cloud [description]
 * @param  {[type]} start [description]
 * @return {[type]}       [description]
 */
PointCloud.prototype.distanceTo = function(cloud, start) {

	var sum = 0, numPoints = this.points.length,
		i = start, j, k, len,
		weight, distance,
		matched = [],
		index = -1,
		min = +Infinity;

	for (k = 0; k < numPoints; k++) { matched[k] = false; }

	do {

		index = -1;
		min = +Infinity;

		for (j = 0, len = matched.length; j < len; j++) {

			if (matched[j]) { continue; }

			distance = this.points[i].distanceTo( cloud.points[j] );

			if (distance < min) {
				min = distance;
				index = j;
			}
		}

		matched[index] = true;
		weight = 1 - ((i - start + numPoints) % numPoints) / numPoints;
		sum += weight * min;
		i = (i + 1) % numPoints;

	} while (i != start);

	return sum;
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
 * register mouse/touch events on an element
 * @param {HTMLElement} target which element to register event listeners on
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

	// for each point cloud
	for (var i = 0; i < this.pointClouds.length; i++) {
		var d = this.greedyCloudMatch(cloud, this.pointClouds[i]);
		if (d < b) {
			b = d;
			u = i;
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
 * something something clouds
 * @param {PointCloud} cloud1 a point cloud
 * @param {PointCloud} cloud2 another point cloud
 * @return {Number} the smallest distance between clouds
 */
SymbolRecognizer.prototype.greedyCloudMatch = function(cloud1, cloud2) {

	var i, len = cloud1.points.length,
		step = Math.floor(Math.pow(len, 0.5)),
		min = +Infinity, distance1, distance2;

	for (i = 0; i < len; i += step) {

		distance1 = cloud1.distanceTo(cloud2, i);
		distance2 = cloud2.distanceTo(cloud1, i);

		min = Math.min(min, Math.min(distance1, distance2));
	}

	return min;
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