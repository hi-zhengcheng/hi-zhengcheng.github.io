(this.webpackJsonpthree=this.webpackJsonpthree||[]).push([[0],{51:function(e,t,n){},58:function(e,t,n){"use strict";n.r(t);var a=n(7),i=n(1),r=n.n(i),o=n(22),d=n.n(o),s=(n(51),n(2)),l=n(6),c=n(4),h=n(5),u=n(10),p=n(0),m=n(23),v=n(29),b=function(e){Object(c.a)(n,e);var t=Object(h.a)(n);function n(e){var a;return Object(s.a)(this,n),(a=t.call(this,e)).rootDiv=r.a.createRef(),a.controllerDiv=r.a.createRef(),a}return Object(l.a)(n,[{key:"componentDidMount",value:function(){var e={speedX:0,speedY:0,speedZ:0,lightX:40,lightY:10,lightZ:40,materialColor:65280,texture:null,showTexture:!0},t=new p.X;t.setSize(window.innerWidth,window.innerHeight),this.rootDiv.appendChild(t.domElement);var n=new p.F(75,window.innerWidth/window.innerHeight,.1,1e3);n.position.set(2,1,10),n.lookAt(0,0,0);var a=new p.L,i=new p.d(1,1,1),r=new p.C({color:e.materialColor});(new p.Q).load("./wall.jpg",(function(t){e.texture=t,e.showTexture&&(r.map=t,r.needsUpdate=!0)}));var o=new p.A(i,r);a.add(o);var d=new p.G(16777215);d.position.set(e.lightX,e.lightY,e.lightZ),a.add(d);var s=new p.b(5);a.add(s);var l=new v.a(n,t.domElement);!function i(){requestAnimationFrame(i),l.update(),o.rotation.x+=e.speedX,o.rotation.y+=e.speedY,o.rotation.z+=e.speedZ,n.updateProjectionMatrix(),t.render(a,n)}();var c=new m.a.gui.GUI({autoPlace:!1}),h=c.addFolder("camera projection");h.add(n,"fov").min(30).max(120).step(5),h.open();var u=c.addFolder("light position");u.add(d.position,"x").min(-40).max(40).step(5),u.add(d.position,"y").min(-40).max(40).step(5),u.add(d.position,"z").min(-40).max(40).step(5),u.open();var b=c.addFolder("rotate speed");b.add(e,"speedX").min(0).max(.1).step(.005),b.add(e,"speedY").min(0).max(.1).step(.005),b.add(e,"speedZ").min(0).max(.1).step(.005),b.open();var f=c.addFolder("cube");f.addColor(e,"materialColor").name("color").onChange((function(){r.color.set(e.materialColor)})),f.add(r,"wireframe"),f.add(e,"showTexture").onChange((function(t){r.map=t?e.texture:null,r.needsUpdate=!0})),f.open();var j=c.addFolder("show axes");j.add(s,"visible"),j.open(),this.controllerDiv.append(c.domElement)}},{key:"render",value:function(){var e=this;return Object(a.jsxs)("div",{style:{position:"absolute",top:0},children:[Object(a.jsx)("div",{ref:function(t){return e.rootDiv=t}}),Object(a.jsx)("div",{ref:function(t){return e.controllerDiv=t},style:{position:"absolute",right:0,top:0}})]})}}]),n}(i.Component),f=n(26),j=function(e){Object(c.a)(n,e);var t=Object(h.a)(n);function n(e){var a;return Object(s.a)(this,n),(a=t.call(this,e)).rootDiv=r.a.createRef(),a.controllerDiv=r.a.createRef(),a}return Object(l.a)(n,[{key:"componentDidMount",value:function(){var e={speedX:.001,speedY:.001,speedZ:0,lightX:40,lightY:10,lightZ:40,materialColor:16777215,texture:null,showTexture:!0},t=new p.X;t.setSize(window.innerWidth,window.innerHeight),this.rootDiv.appendChild(t.domElement);var n=new p.F(45,window.innerWidth/window.innerHeight,.1,1e3);n.position.set(0,0,1.5),n.lookAt(0,0,0);var a=new p.L,i=new p.N(.5,32,32),r=new p.D({color:e.materialColor});(new p.Q).load("./earth/2_no_clouds_8k.jpg",(function(t){e.texture=t,e.showTexture&&(r.map=t,r.needsUpdate=!0)}));var o=new p.A(i,r);a.add(o),a.add(new p.a(6710886));var d=new p.i(16777215,1);d.position.set(5,3,5),a.add(d);var s=new p.b(5);a.add(s);var l=new f.a(n,t.domElement);!function i(){requestAnimationFrame(i),l.update(),o.rotation.x+=e.speedX,o.rotation.y+=e.speedY,o.rotation.z+=e.speedZ,n.updateProjectionMatrix(),t.render(a,n)}();var c=new m.a.gui.GUI({autoPlace:!1}),h=c.addFolder("camera projection");h.add(n,"fov").min(30).max(120).step(5),h.open();var u=c.addFolder("light position");u.add(d.position,"x").min(-40).max(40).step(5),u.add(d.position,"y").min(-40).max(40).step(5),u.add(d.position,"z").min(-40).max(40).step(5),u.open();var v=c.addFolder("rotate speed");v.add(e,"speedX").min(0).max(.1).step(.005),v.add(e,"speedY").min(0).max(.1).step(.005),v.add(e,"speedZ").min(0).max(.1).step(.005),v.open();var b=c.addFolder("earth");b.addColor(e,"materialColor").name("color").onChange((function(){r.color.set(e.materialColor)})),b.add(r,"wireframe"),b.add(e,"showTexture").onChange((function(t){r.map=t?e.texture:null,r.needsUpdate=!0})),b.open();var j=c.addFolder("show axes");j.add(s,"visible"),j.open(),this.controllerDiv.append(c.domElement)}},{key:"render",value:function(){var e=this;return Object(a.jsxs)("div",{style:{position:"absolute",top:0},children:[Object(a.jsx)("div",{ref:function(t){return e.rootDiv=t}}),Object(a.jsx)("div",{ref:function(t){return e.controllerDiv=t},style:{position:"absolute",right:0,top:0}})]})}}]),n}(i.Component),w=n(34),g=n(41),x=n(30),O=n(76),y=n(77),C=n(78),F=function(e){Object(c.a)(n,e);var t=Object(h.a)(n);function n(){return Object(s.a)(this,n),t.apply(this,arguments)}return Object(l.a)(n,[{key:"render",value:function(){var e,t=[],n=Object(g.a)([{path:"/cube",primary:"Cube"},{path:"/earth",primary:"Earth"},{path:"/airplane",primary:"Airplane"},{path:"/pointcloud",primary:"Pointcloud"},{path:"/globe",primary:"Globe"}].entries());try{for(n.s();!(e=n.n()).done;){var i=Object(w.a)(e.value,2),r=i[0],o=i[1];t.push(Object(a.jsx)(y.a,{button:!0,component:x.b,to:o.path,children:Object(a.jsx)(C.a,{primary:o.primary})},r))}}catch(d){n.e(d)}finally{n.f()}return Object(a.jsx)(O.a,{component:"nav","aria-label":"secondary mailbox folders",children:t})}}]),n}(i.Component),_=n(42),D=n(38),k=function(e){Object(c.a)(n,e);var t=Object(h.a)(n);function n(e){var a;return Object(s.a)(this,n),(a=t.call(this,e)).rootDiv=null,a.controllerDiv=null,a.currentObj3d=null,a.materials=null,a.scene=null,a.renderer=null,a.directLight=null,a.ambientLight=null,a.paramObj={scale:1,translateX:0,translateY:0,translateZ:0,bg_color:16777215,direct_light_color:16777215,ambient_light_color:1123942},a}return Object(l.a)(n,[{key:"componentDidMount",value:function(){var e=new p.X;e.setSize(window.innerWidth,window.innerHeight),e.setClearColor(this.paramObj.bg_color,1),this.rootDiv.appendChild(e.domElement),this.renderer=e;var t=new p.F(45,window.innerWidth/window.innerHeight,.1,1e4);t.position.set(10,5,1e3),t.lookAt(0,0,0);var n=new p.L;this.scene=n,(new _.a).load("./modelnet40/airplane_0726_SMPL.mtl",function(e){this.materials=e,e.preload(),(new D.a).setMaterials(e).load("./modelnet40/airplane_0726_SMPL.obj",function(e){e.scale.set(this.paramObj.scale,this.paramObj.scale,this.paramObj.scale),e.position.set(this.paramObj.translateX,this.paramObj.translateY,this.paramObj.translateZ),this.scene.add(e),this.currentObj3d=e}.bind(this))}.bind(this));var a=new p.a(this.paramObj.ambient_light_color);n.add(a),this.ambientLight=a;var i=new p.i(this.paramObj.direct_light_color,1);i.position.set(5,3,5),n.add(i),this.directLight=i;var r=new p.b(2e3);n.add(r);var o=new f.a(t,e.domElement);!function a(){requestAnimationFrame(a),o.update(),t.updateProjectionMatrix(),e.render(n,t)}();var d=new m.a.gui.GUI({autoPlace:!1});d.addColor(this.paramObj,"bg_color").name("bg color").onChange(function(e){this.renderer.setClearColor(e,1)}.bind(this)),d.addColor(this.paramObj,"ambient_light_color").name("ambient").onChange(function(e){this.ambientLight.color.set(e)}.bind(this));var s=d.addFolder("camera projection");s.add(t,"fov").min(30).max(120).step(5),s.open();var l=d.addFolder("light position");l.add(i.position,"x").min(-40).max(40).step(5),l.add(i.position,"y").min(-40).max(40).step(5),l.add(i.position,"z").min(-40).max(40).step(5),l.addColor(this.paramObj,"direct_light_color").name("color").onChange(function(e){this.directLight.color.set(e)}.bind(this)),l.open();var c=d.addFolder("object");c.add(this.paramObj,"scale").min(.001).max(10).step(.001).onChange(function(e){this.currentObj3d&&this.currentObj3d.scale.set(e,e,e)}.bind(this)),c.add(this.paramObj,"translateX").min(-1e3).max(1e3).step(1).onChange(function(e){this.currentObj3d&&this.currentObj3d.position.setX(e)}.bind(this)),c.add(this.paramObj,"translateY").min(-1e3).max(1e3).step(1).onChange(function(e){this.currentObj3d&&this.currentObj3d.position.setY(e)}.bind(this)),c.add(this.paramObj,"translateZ").min(-1e3).max(1e3).step(1).onChange(function(e){this.currentObj3d&&this.currentObj3d.position.setZ(e)}.bind(this)),c.open();var h=d.addFolder("axes");h.add(r,"visible"),h.open();var u=d.addFolder("upload"),v={loadFile:function(){document.getElementById("uploadButton").click()}};u.add(v,"loadFile").name("Load obj file"),u.open(),this.controllerDiv.append(d.domElement)}},{key:"newFileCallback",value:function(){var e=this,t=document.getElementById("uploadButton").files[0],n=new FileReader;n.addEventListener("load",(function(t){var n=t.target.result,a=(new D.a).setMaterials(e.materials).parse(n);a?(a.scale.set(e.paramObj.scale,e.paramObj.scale,e.paramObj.scale),a.position.set(e.paramObj.translateX,e.paramObj.translateY,e.paramObj.translateZ),e.scene.add(a),e.scene.remove(e.currentObj3d),e.currentObj3d=a,alert("success")):alert("Load error")}),!1),n.readAsText(t)}},{key:"render",value:function(){var e=this;return Object(a.jsxs)("div",{style:{position:"absolute",top:0},children:[Object(a.jsx)("div",{ref:function(t){return e.rootDiv=t}}),Object(a.jsx)("div",{ref:function(t){return e.controllerDiv=t},style:{position:"absolute",right:0,top:0}}),Object(a.jsx)("input",{id:"uploadButton",type:"file",style:{visibility:"hidden"},onChange:this.newFileCallback.bind(this)})]})}}]),n}(i.Component),E=function(e){Object(c.a)(n,e);var t=Object(h.a)(n);function n(e){var a;return Object(s.a)(this,n),(a=t.call(this,e)).rootDiv=null,a.controllerDiv=null,a.geometry=null,a.material=null,a.current_obj=null,a.scene=null,a.renderer=null,a.paramObj={bg_color:0,camera_z:5,point_radius:.004,materialColor:4286945},a}return Object(l.a)(n,[{key:"componentDidMount",value:function(){var e=this,t=new p.X;t.setSize(window.innerWidth,window.innerHeight),t.setClearColor(this.paramObj.bg_color,1),this.rootDiv.appendChild(t.domElement),this.renderer=t;var n=new p.F(45,window.innerWidth/window.innerHeight,.1,1e3);n.position.set(2,5,this.paramObj.camera_z),n.lookAt(0,0,0);var a=new p.L;this.scene=a,this.geometry=new p.o(this.paramObj.point_radius,3),this.material=new p.D({color:this.paramObj.materialColor}),a.add(new p.a(16777215));var i=new p.b(2e3);a.add(i);var r=new f.a(n,t.domElement);!function e(){requestAnimationFrame(e),r.update(),n.updateProjectionMatrix(),t.render(a,n)}(),fetch("./pointcloud/vase.txt").then((function(e){return e.text()})).then((function(t){var n=e.parseContent(t);e.showPointCloud(n)}));var o=new m.a.gui.GUI({autoPlace:!1}),d=o.addFolder("camera projection");d.add(n,"fov").min(30).max(120).step(5),d.open();var s=o.addFolder("object");s.addColor(this.paramObj,"materialColor").name("point color").onChange(function(e){this.material.color.set(e)}.bind(this)),s.addColor(this.paramObj,"bg_color").name("bg color").onChange(function(e){this.renderer.setClearColor(e,1)}.bind(this)),s.open();var l=o.addFolder("axes");l.add(i,"visible"),l.open();var c=o.addFolder("upload"),h={loadFile:function(){document.getElementById("uploadButton").click()}};c.add(h,"loadFile").name("Load obj file"),c.open(),this.controllerDiv.append(o.domElement)}},{key:"loadtxt",value:function(e,t){var n=new FileReader;n.onload=function(e){var n=e.target.result,a=this.parseContent(n);t(a)}.bind(this),n.readAsText(e)}},{key:"showPointCloud",value:function(e){this.current_obj&&this.scene.remove(this.current_obj);for(var t=new p.p(this.geometry,this.material,e.length),n=new p.z,a=0;a<e.length;a++)n.setPosition(e[a][0],e[a][1],e[a][2]),t.setMatrixAt(a,n);this.scene.add(t),this.current_obj=t}},{key:"parseContent",value:function(e){var t,n,a=e.split("\n"),i=a[0];if(t=this.parseLine(i)){n=t.length;for(var r,o,d,s,l,c,h,u,p,m,v,b,f=[],j=0;j<a.length;j++)(t=this.parseLine(a[j]))&&t.length===n&&f.push(t);r=f[0][0],o=f[0][1],d=f[0][2],s=f[0][0],l=f[0][1],c=f[0][2],h=0,u=0,p=0;for(var w=0;w<f.length;w++)r<f[w][0]&&(r=f[w][0]),s>f[w][0]&&(s=f[w][0]),h+=f[w][0],o<f[w][1]&&(o=f[w][1]),l>f[w][1]&&(l=f[w][1]),u+=f[w][1],d<f[w][2]&&(d=f[w][2]),c>f[w][2]&&(c=f[w][2]),p+=f[w][2];m=h/f.length,v=u/f.length,b=p/f.length;var g=r-s;g=(g=(g=g?g>=o-l:o-l)?g>=d-c:d-c)?g>0:1;for(var x=0;x<f.length;x++)f[x][0]=(f[x][0]-m)/g,f[x][1]=(f[x][1]-v)/g,f[x][2]=(f[x][2]-b)/g;return f}alert("Parse Error")}},{key:"parseLine",value:function(e){if(!e)return null;for(var t=[" ",",",", "],n=0;n<t.length;n++){var a=e.split(t[n]);if(3===a.length||6===a.length){for(var i=0;i<a.length;i++)a[i]=parseFloat(a[i]);return a}}return null}},{key:"handlerFileSelected",value:function(e){if(e.target.files&&e.target.files[0]){var t=e.target.files[0];this.loadtxt(t,this.showPointCloud.bind(this))}else alert("No file uploaded")}},{key:"render",value:function(){var e=this;return Object(a.jsxs)("div",{style:{position:"absolute",top:0},children:[Object(a.jsx)("div",{ref:function(t){return e.rootDiv=t}}),Object(a.jsx)("div",{ref:function(t){return e.controllerDiv=t},style:{position:"absolute",right:0,top:0}}),Object(a.jsx)("input",{id:"uploadButton",type:"file",style:{visibility:"hidden"},onChange:this.handlerFileSelected.bind(this)})]})}}]),n}(i.Component),L=n(39),A=n.n(L),M=n(43),P=n(44),R=function(e){Object(c.a)(n,e);var t=Object(h.a)(n);function n(e){var a;return Object(s.a)(this,n),(a=t.call(this,e)).canvas2d=null,a.rootDiv=null,a.controllerDiv=null,a.scene=null,a.camera=null,a.renderer=null,a.controls=null,a.renderRequested=!1,a}return Object(l.a)(n,[{key:"componentDidMount",value:function(){var e=new p.X;e.setSize(window.innerWidth,window.innerHeight),e.setClearColor("#000000",1),this.rootDiv.appendChild(e.domElement),this.renderer=e;var t=new p.L;t.background=new p.g("black"),this.scene=t;var n=(new p.Q).load("./globe/world.jpg"),a=new p.M(1,64,32),i=new p.B({map:n});t.add(new p.A(a,i));var r=new p.F(60,window.innerWidth/window.innerHeight,.1,10);r.position.set(0,0,2.5),r.lookAt(0,0,0),this.camera=r;var o=new v.a(r,e.domElement);o.enableDamping=!0,o.enablePan=!1,o.minDistance=1.2,o.maxDistance=4,o.update(),this.controls=o,this.loadFile("./globe/gpw_v4_basic_demographic_characteristics_rev10_a000_014mt_2010_cntm_1_deg.asc").then(this.parseData.bind(this)).then(this.addBoxes.bind(this)).then(this.render_3d.bind(this)),o.addEventListener("change",this.requestRenderIfNotRequested.bind(this)),window.addEventListener("resize",this.requestRenderIfNotRequested.bind(this))}},{key:"render_3d",value:function(){this.renderRequested=!1,this.resizeRendererToDisplaySize()&&(this.camera.aspect=window.innerWidth/window.innerHeight,this.camera.updateProjectionMatrix()),this.controls.update(),this.renderer.render(this.scene,this.camera)}},{key:"resizeRendererToDisplaySize",value:function(){var e=this.renderer.domElement,t=e.style.width!==window.innerWidth||e.style.height!==window.innerHeight;return t&&(e.style.width=this.px(window.innerWidth),e.style.height=this.px(window.innerHeight),this.renderer.setSize(window.innerWidth,window.innerHeight,!1)),t}},{key:"requestRenderIfNotRequested",value:function(){this.renderRequested||(this.renderRequested=!0,requestAnimationFrame(this.render_3d.bind(this)))}},{key:"loadFile",value:function(){var e=Object(M.a)(A.a.mark((function e(t){var n;return A.a.wrap((function(e){for(;;)switch(e.prev=e.next){case 0:return e.next=2,fetch(t);case 2:return n=e.sent,e.abrupt("return",n.text());case 4:case"end":return e.stop()}}),e)})));return function(t){return e.apply(this,arguments)}}()},{key:"parseData",value:function(e){var t,n,a=[],i={data:a};return e.split("\n").forEach((function(e){var r=e.trim().split(/\s+/);if(2===r.length)i[r[0]]=parseFloat(r[1]);else if(r.length>2){var o=r.map((function(e){var a=parseFloat(e);if(a!==i.NODATA_value)return t=Math.max(void 0===t?a:t,a),n=Math.min(void 0===n?a:n,a),a}));a.push(o)}})),Object.assign(i,{min:n,max:t})}},{key:"drawData",value:function(e){var t=this,n=e.min,a=e.max,i=e.ncols,r=e.nrows,o=e.data,d=a-n,s=this.canvas2d.getContext("2d");s.canvas.width=i,s.canvas.height=r,s.canvas.style.width=this.px(2*i),s.canvas.style.height=this.px(2*r),s.fillStyle="#444",s.fillRect(0,0,s.canvas.width,s.canvas.height),o.forEach((function(e,a){e.forEach((function(e,i){if(void 0!==e){var r=(e-n)/d;s.fillStyle=t.hsl(1,1,r),s.fillRect(i,a,1,1)}}))}))}},{key:"px",value:function(e){return"".concat(0|e,"px")}},{key:"hsl",value:function(e,t,n){return"hsl(".concat(360*e|0,",").concat(100*t|0,"%,").concat(100*n|0,"%)")}},{key:"addBoxes",value:function(e){var t=e.min,n=e.max,a=e.data,i=n-t,r=new p.E;this.scene.add(r);var o=new p.E;r.add(o);var d=new p.E;d.position.z=1,o.add(d);var s=new p.E;s.position.z=.5,d.add(s);var l=new p.g,c=.5*Math.PI,h=-.135*Math.PI,u=[];a.forEach((function(n,a){n.forEach((function(n,m){if(void 0!==n){var v=(n-t)/i,b=new p.c(1,1,1);r.rotation.y=p.y.degToRad(m+e.xllcorner)+c,o.rotation.x=p.y.degToRad(a+e.yllcorner)+h,d.scale.set(.005,.005,p.y.lerp(.01,.5,v)),s.updateWorldMatrix(!0,!1),b.applyMatrix4(s.matrixWorld);var f=p.y.lerp(.7,.3,v),j=p.y.lerp(.4,.1,v);l.setHSL(f,1,j);var w=l.toArray().map((function(e){return 255*e})),g=b.getAttribute("position").count,x=new Uint8Array(3*g);x.forEach((function(e,t){x[t]=w[t%3]}));var O=new p.e(x,3,!0);b.setAttribute("color",O),u.push(b)}}))}));var m=P.a.mergeBufferGeometries(u,!1),v=new p.B({vertexColors:p.W}),b=new p.A(m,v);this.scene.add(b)}},{key:"render",value:function(){var e=this;return Object(a.jsxs)("div",{style:{position:"absolute",top:0},children:[Object(a.jsx)("div",{ref:function(t){return e.rootDiv=t}}),Object(a.jsx)("div",{ref:function(t){return e.controllerDiv=t},style:{position:"absolute",right:0,top:0}})]})}}]),n}(i.Component),z=function(e){Object(c.a)(n,e);var t=Object(h.a)(n);function n(){return Object(s.a)(this,n),t.apply(this,arguments)}return Object(l.a)(n,[{key:"render",value:function(){return Object(a.jsx)("div",{children:Object(a.jsxs)(u.c,{children:[Object(a.jsx)(u.a,{exact:!0,path:"/",component:F}),Object(a.jsx)(u.a,{path:"/cube",component:b}),Object(a.jsx)(u.a,{path:"/earth",component:j}),Object(a.jsx)(u.a,{path:"/airplane",component:k}),Object(a.jsx)(u.a,{path:"/pointcloud",component:E}),Object(a.jsx)(u.a,{path:"/globe",component:R})]})})}}]),n}(i.Component),X=function(e){e&&e instanceof Function&&n.e(3).then(n.bind(null,79)).then((function(t){var n=t.getCLS,a=t.getFID,i=t.getFCP,r=t.getLCP,o=t.getTTFB;n(e),a(e),i(e),r(e),o(e)}))};d.a.render(Object(a.jsx)(r.a.StrictMode,{children:Object(a.jsx)(x.a,{basename:window.location.pathname,children:Object(a.jsx)(z,{})})}),document.getElementById("root")),X()}},[[58,1,2]]]);