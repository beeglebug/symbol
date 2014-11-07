/**
 * Recognizer
 * @constructor
 */
var Recognizer = function() {

	this.pointClouds = [];
	this.listeners = {};

	this.registerEvents(document);
};


/**
 * register mouse/touch events on an element
 * @param {HTMLElement} target which element to register event listeners on
 */
Recognizer.prototype.registerEvents = function(target) {

	var drawing = false;
	var symbol = [];
	var line = 0;
	var self = this;

	target.addEventListener('mousedown', function() {
		drawing = true;
		symbol = [];
		line++;
	});

	target.addEventListener('mousemove', function(e) {
		if(drawing) {
			symbol.push([e.offsetX, e.offsetY]);
		}
	});

	target.addEventListener('mouseup', function() {
		drawing = false;
		self.recognize([symbol]);
	});
};


/**
 * [on description]
 * @param  {[type]}   name     [description]
 * @param  {Function} callback [description]
 * @return {[type]}            [description]
 */
Recognizer.prototype.on = function(name, callback) {

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
Recognizer.prototype.recognize = function(pointsList) {

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
Recognizer.prototype.add = function(name, pointList) {

	this.pointClouds.push( new PointCloud(name, pointList) );

};


/**
 * something something clouds
 * @param {PointCloud} cloud1 a point cloud
 * @param {PointCloud} cloud2 another point cloud
 * @return {Number} the smallest distance between clouds
 */
Recognizer.prototype.greedyCloudMatch = function(cloud1, cloud2) {

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
