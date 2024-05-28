(self.webpackChunk_N_E=self.webpackChunk_N_E||[]).push([[157],{2500:function(e,t,r){(window.__NEXT_P=window.__NEXT_P||[]).push(["/run_model",function(){return r(4674)}])},4674:function(e,t,r){"use strict";r.r(t),r.d(t,{default:function(){return d}});var i=r(5893),n=r(1664),o=r.n(n),s=r(5185);r(7294);var a=r(4970),c=r.n(a);function d(){return(0,i.jsxs)("div",{children:[(0,i.jsx)("h2",{children:"Run Model"}),(0,i.jsx)(s.A,{radius:"full",className:"bg-gradient-to-tr from-pink-500 to-yellow-500 text-white shadow-lg",children:"Button"}),(0,i.jsx)("div",{children:(0,i.jsx)(c(),{})}),(0,i.jsx)("br",{}),(0,i.jsx)(o(),{href:"/",children:"go home"})]})}},4970:function(e,t,r){var i;e.exports=(i=r(7294),function(e){var t={};function r(i){if(t[i])return t[i].exports;var n=t[i]={i:i,l:!1,exports:{}};return e[i].call(n.exports,n,n.exports,r),n.l=!0,n.exports}return r.m=e,r.c=t,r.d=function(e,t,i){r.o(e,t)||Object.defineProperty(e,t,{enumerable:!0,get:i})},r.r=function(e){"undefined"!=typeof Symbol&&Symbol.toStringTag&&Object.defineProperty(e,Symbol.toStringTag,{value:"Module"}),Object.defineProperty(e,"__esModule",{value:!0})},r.t=function(e,t){if(1&t&&(e=r(e)),8&t||4&t&&"object"==typeof e&&e&&e.__esModule)return e;var i=Object.create(null);if(r.r(i),Object.defineProperty(i,"default",{enumerable:!0,value:e}),2&t&&"string"!=typeof e)for(var n in e)r.d(i,n,(function(t){return e[t]}).bind(null,n));return i},r.n=function(e){var t=e&&e.__esModule?function(){return e.default}:function(){return e};return r.d(t,"a",t),t},r.o=function(e,t){return Object.prototype.hasOwnProperty.call(e,t)},r.p="",r(r.s="./src/react-webcam.tsx")}({"./src/react-webcam.tsx":function(e,t,r){"use strict";r.r(t);var i,n=r(/*! react */"react"),o=(i=function(e,t){return(i=Object.setPrototypeOf||({__proto__:[]})instanceof Array&&function(e,t){e.__proto__=t}||function(e,t){for(var r in t)t.hasOwnProperty(r)&&(e[r]=t[r])})(e,t)},function(e,t){function r(){this.constructor=e}i(e,t),e.prototype=null===t?Object.create(t):(r.prototype=t.prototype,new r)}),s=function(){return(s=Object.assign||function(e){for(var t,r=1,i=arguments.length;r<i;r++)for(var n in t=arguments[r])Object.prototype.hasOwnProperty.call(t,n)&&(e[n]=t[n]);return e}).apply(this,arguments)},a=function(e,t){var r={};for(var i in e)Object.prototype.hasOwnProperty.call(e,i)&&0>t.indexOf(i)&&(r[i]=e[i]);if(null!=e&&"function"==typeof Object.getOwnPropertySymbols)for(var n=0,i=Object.getOwnPropertySymbols(e);n<i.length;n++)0>t.indexOf(i[n])&&Object.prototype.propertyIsEnumerable.call(e,i[n])&&(r[i[n]]=e[i[n]]);return r};function c(){return!!(navigator.mediaDevices&&navigator.mediaDevices.getUserMedia)}"undefined"!=typeof window&&(void 0===navigator.mediaDevices&&(navigator.mediaDevices={}),void 0===navigator.mediaDevices.getUserMedia&&(navigator.mediaDevices.getUserMedia=function(e){var t=navigator.getUserMedia||navigator.webkitGetUserMedia||navigator.mozGetUserMedia||navigator.msGetUserMedia;return t?new Promise(function(r,i){t.call(navigator,e,r,i)}):Promise.reject(Error("getUserMedia is not implemented in this browser"))}));var d=function(e){function t(t){var r=e.call(this,t)||this;return r.canvas=null,r.ctx=null,r.requestUserMediaId=0,r.unmounted=!1,r.state={hasUserMedia:!1},r}return o(t,e),t.prototype.componentDidMount=function(){var e=this.state,t=this.props;if(this.unmounted=!1,!c()){t.onUserMediaError("getUserMedia not supported");return}e.hasUserMedia||this.requestUserMedia(),t.children&&"function"!=typeof t.children&&console.warn("children must be a function")},t.prototype.componentDidUpdate=function(e){var t=this.props;if(!c()){t.onUserMediaError("getUserMedia not supported");return}var r=JSON.stringify(e.audioConstraints)!==JSON.stringify(t.audioConstraints),i=JSON.stringify(e.videoConstraints)!==JSON.stringify(t.videoConstraints),n=e.minScreenshotWidth!==t.minScreenshotWidth,o=e.minScreenshotHeight!==t.minScreenshotHeight;(i||n||o)&&(this.canvas=null,this.ctx=null),(r||i)&&(this.stopAndCleanup(),this.requestUserMedia())},t.prototype.componentWillUnmount=function(){this.unmounted=!0,this.stopAndCleanup()},t.stopMediaStream=function(e){e&&(e.getVideoTracks&&e.getAudioTracks?(e.getVideoTracks().map(function(t){e.removeTrack(t),t.stop()}),e.getAudioTracks().map(function(t){e.removeTrack(t),t.stop()})):e.stop())},t.prototype.stopAndCleanup=function(){var e=this.state;e.hasUserMedia&&(t.stopMediaStream(this.stream),e.src&&window.URL.revokeObjectURL(e.src))},t.prototype.getScreenshot=function(e){var t=this.state,r=this.props;if(!t.hasUserMedia)return null;var i=this.getCanvas(e);return i&&i.toDataURL(r.screenshotFormat,r.screenshotQuality)},t.prototype.getCanvas=function(e){var t=this.state,r=this.props;if(!this.video||!t.hasUserMedia||!this.video.videoHeight)return null;if(!this.ctx){var i=this.video.videoWidth,n=this.video.videoHeight;if(!this.props.forceScreenshotSourceSize){var o=i/n;n=(i=r.minScreenshotWidth||this.video.clientWidth)/o,r.minScreenshotHeight&&n<r.minScreenshotHeight&&(i=(n=r.minScreenshotHeight)*o)}this.canvas=document.createElement("canvas"),this.canvas.width=(null==e?void 0:e.width)||i,this.canvas.height=(null==e?void 0:e.height)||n,this.ctx=this.canvas.getContext("2d")}var s=this.ctx,a=this.canvas;return s&&a&&(a.width=(null==e?void 0:e.width)||a.width,a.height=(null==e?void 0:e.height)||a.height,r.mirrored&&(s.translate(a.width,0),s.scale(-1,1)),s.imageSmoothingEnabled=r.imageSmoothing,s.drawImage(this.video,0,0,(null==e?void 0:e.width)||a.width,(null==e?void 0:e.height)||a.height),r.mirrored&&(s.scale(-1,1),s.translate(-a.width,0))),a},t.prototype.requestUserMedia=function(){var e=this,r=this.props,i=function(i,n){var o={video:void 0===n||n};r.audio&&(o.audio=void 0===i||i),e.requestUserMediaId++;var s=e.requestUserMediaId;navigator.mediaDevices.getUserMedia(o).then(function(r){e.unmounted||s!==e.requestUserMediaId?t.stopMediaStream(r):e.handleUserMedia(null,r)}).catch(function(t){e.handleUserMedia(t)})};if("mediaDevices"in navigator)i(r.audioConstraints,r.videoConstraints);else{var n=function(e){return{optional:[{sourceId:e}]}},o=function(e){var t=e.deviceId;return"string"==typeof t?t:Array.isArray(t)&&t.length>0?t[0]:"object"==typeof t&&t.ideal?t.ideal:null};MediaStreamTrack.getSources(function(e){var t=null,s=null;e.forEach(function(e){"audio"===e.kind?t=e.id:"video"===e.kind&&(s=e.id)});var a=o(r.audioConstraints);a&&(t=a);var c=o(r.videoConstraints);c&&(s=c),i(n(t),n(s))})}},t.prototype.handleUserMedia=function(e,t){var r=this.props;if(e||!t){this.setState({hasUserMedia:!1}),r.onUserMediaError(e);return}this.stream=t;try{this.video&&(this.video.srcObject=t),this.setState({hasUserMedia:!0})}catch(e){this.setState({hasUserMedia:!0,src:window.URL.createObjectURL(t)})}r.onUserMedia(t)},t.prototype.render=function(){var e=this,t=this.state,r=this.props,i=r.audio,o=(r.forceScreenshotSourceSize,r.onUserMedia,r.onUserMediaError,r.screenshotFormat,r.screenshotQuality,r.minScreenshotWidth,r.minScreenshotHeight,r.audioConstraints,r.videoConstraints,r.imageSmoothing,r.mirrored),c=r.style,d=void 0===c?{}:c,u=r.children,h=a(r,["audio","forceScreenshotSourceSize","onUserMedia","onUserMediaError","screenshotFormat","screenshotQuality","minScreenshotWidth","minScreenshotHeight","audioConstraints","videoConstraints","imageSmoothing","mirrored","style","children"]),l=o?s(s({},d),{transform:(d.transform||"")+" scaleX(-1)"}):d,f={getScreenshot:this.getScreenshot.bind(this)};return n.createElement(n.Fragment,null,n.createElement("video",s({autoPlay:!0,src:t.src,muted:!i,playsInline:!0,ref:function(t){e.video=t},style:l},h)),u&&u(f))},t.defaultProps={audio:!1,forceScreenshotSourceSize:!1,imageSmoothing:!0,mirrored:!1,onUserMedia:function(){},onUserMediaError:function(){},screenshotFormat:"image/webp",screenshotQuality:.92},t}(n.Component);t.default=d},react:function(e,t){e.exports=i}}).default)}},function(e){e.O(0,[664,185,774,888,179],function(){return e(e.s=2500)}),_N_E=e.O()}]);