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
		scaled.push( this.points[i].clone().subtract(min).divide(size) );
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
		offset = origin.clone().subtract(centroid);

	for (var i = 0, len = this.points.length; i < len; i++) {
		translated.push( this.points[i].clone().add(offset) );
	}

	return translated;
};


/**
 * calculate the center of the cloud
 * @return {Point} a new Point representing the exact center of the cloud
 */
PointCloud.prototype.centroid = function() {

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
