symbol
======

Symbol - A Javascript gesture recognition library

```javascript
// define a symbol
Symbol.add('N', [ [[177,92],[177,2]], [[182,1],[246,95]], [[247,87],[247,1]] ] );

// when the symbol is drawn, trigger a callback
Symbol.on('N', function(b) {
	console.log('an N was drawn');
});
```