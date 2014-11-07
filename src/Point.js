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
 * Create a copy of this Point
 * @returns {Point} a copy of this Point
 */
Point.prototype.clone = function() {
	return new Point(this.x, this.y, this.id);
};


/**
 * [[Description]]
 * @param   {Object}   point [[Description]]
 * @returns {[[Type]]} [[Description]]
 */
Point.prototype.add = function(point) {
	this.x += point.x;
	this.y += point.y;
	return this;
};


/**
 * [[Description]]
 * @param   {Object}   point [[Description]]
 * @returns {[[Type]]} [[Description]]
 */
Point.prototype.subtract = function(point) {
	this.x -= point.x;
	this.y -= point.y;
	return this;
};


/**
 * [[Description]]
 * @param   {[[Type]]} scalar [[Description]]
 * @returns {[[Type]]} [[Description]]
 */
Point.prototype.divide = function(scalar) {
	this.x /= scalar;
	this.y /= scalar;
	return this;
};
