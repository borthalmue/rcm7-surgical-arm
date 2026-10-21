// Minimal faithful THREE math + stubs for headless logic testing
class Vector3{constructor(x=0,y=0,z=0){this.x=x;this.y=y;this.z=z}
 set(x,y,z){this.x=x;this.y=y;this.z=z;return this} clone(){return new Vector3(this.x,this.y,this.z)} copy(v){this.x=v.x;this.y=v.y;this.z=v.z;return this}
 add(v){this.x+=v.x;this.y+=v.y;this.z+=v.z;return this} sub(v){this.x-=v.x;this.y-=v.y;this.z-=v.z;return this}
 addScaledVector(v,s){this.x+=v.x*s;this.y+=v.y*s;this.z+=v.z*s;return this} multiplyScalar(s){this.x*=s;this.y*=s;this.z*=s;return this}
 divideScalar(s){return this.multiplyScalar(1/s)} dot(v){return this.x*v.x+this.y*v.y+this.z*v.z}
 cross(v){const x=this.y*v.z-this.z*v.y,y=this.z*v.x-this.x*v.z,z=this.x*v.y-this.y*v.x;this.x=x;this.y=y;this.z=z;return this}
 length(){return Math.hypot(this.x,this.y,this.z)} normalize(){const l=this.length()||1;return this.divideScalar(l)}
 distanceTo(v){return Math.hypot(this.x-v.x,this.y-v.y,this.z-v.z)} distanceToSquared(v){return (this.x-v.x)**2+(this.y-v.y)**2+(this.z-v.z)**2}
 applyMatrix4(m){const e=m.elements,x=this.x,y=this.y,z=this.z;this.x=e[0]*x+e[4]*y+e[8]*z+e[12];this.y=e[1]*x+e[5]*y+e[9]*z+e[13];this.z=e[2]*x+e[6]*y+e[10]*z+e[14];return this}
 project(){return this}}
class Matrix4{constructor(){this.elements=[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]}
 set(n11,n12,n13,n14,n21,n22,n23,n24,n31,n32,n33,n34,n41,n42,n43,n44){const e=this.elements;e[0]=n11;e[4]=n12;e[8]=n13;e[12]=n14;e[1]=n21;e[5]=n22;e[9]=n23;e[13]=n24;e[2]=n31;e[6]=n32;e[10]=n33;e[14]=n34;e[3]=n41;e[7]=n42;e[11]=n43;e[15]=n44;return this}
 clone(){const m=new Matrix4();m.elements=this.elements.slice();return m} copy(m){this.elements=m.elements.slice();return this}
 makeTranslation(x,y,z){return this.set(1,0,0,x,0,1,0,y,0,0,1,z,0,0,0,1)}
 makeRotationAxis(a,ang){const c=Math.cos(ang),s=Math.sin(ang),t=1-c,x=a.x,y=a.y,z=a.z,tx=t*x,ty=t*y;
  return this.set(tx*x+c,tx*y-s*z,tx*z+s*y,0, tx*y+s*z,ty*y+c,ty*z-s*x,0, tx*z-s*y,ty*z+s*x,t*z*z+c,0, 0,0,0,1)}
 makeBasis(x,y,z){return this.set(x.x,y.x,z.x,0,x.y,y.y,z.y,0,x.z,y.z,z.z,0,0,0,0,1)}
 setPosition(v){const e=this.elements;e[12]=v.x;e[13]=v.y;e[14]=v.z;return this}
 multiply(m){return this.multiplyMatrices(this,m)}
 multiplyMatrices(a,b){const ae=a.elements,be=b.elements,te=new Array(16);
  for(let i=0;i<4;i++)for(let j=0;j<4;j++){let s=0;for(let k=0;k<4;k++)s+=ae[k*4+i]*be[j*4+k];te[j*4+i]=s}this.elements=te;return this}}
class Obj{constructor(){this.children=[];this.matrix=new Matrix4();this.position=new Vector3();this.scale=new Vector3(1,1,1);this.rotation={x:0,y:0,z:0,set(a,b,c){this.x=a;this.y=b;this.z=c}};this.quaternion={setFromUnitVectors(){return this}};this.visible=true;this.userData={}}
 add(o){this.children.push(o);return this} updateMatrixWorld(){} }
class Mesh extends Obj{constructor(g,m){super();this.geometry=g;this.material=m}}
class Line extends Obj{constructor(g,m){super();this.geometry=g;this.material=m} computeLineDistances(){return this}}
class BufferGeometry{constructor(){this.attributes={}} setAttribute(k,a){this.attributes[k]=a;return this} setFromPoints(p){this.pts=p;return this} setDrawRange(a,b){this.dr=[a,b]}}
class BufferAttribute{constructor(a,n,norm){this.array=a;this.itemSize=n;this.normalized=!!norm}}
class Mat{constructor(p={}){Object.assign(this,p);this.color=p.color;}}
class Color{constructor(v){this.v=v}}
class Curve{}
global.THREE={Vector3,Matrix4,Group:Obj,Object3D:Obj,Scene:class extends Obj{updateMatrixWorld(){}},Mesh,Line,BufferGeometry,BufferAttribute,
 MeshStandardMaterial:Mat,MeshBasicMaterial:Mat,LineDashedMaterial:Mat,LineBasicMaterial:Mat,Color,Curve,
 PerspectiveCamera:class extends Obj{constructor(){super();this.aspect=1} updateProjectionMatrix(){}},
 WebGLRenderer:class{constructor(){} setPixelRatio(){} getPixelRatio(){return 1} setSize(){} render(){}},
 OrbitControls:class{constructor(){this.target=new Vector3()} update(){return false} addEventListener(){}},
 HemisphereLight:Obj,DirectionalLight:class extends Obj{},GridHelper:class extends Obj{constructor(){super();this.material={}}},
 SphereGeometry:BufferGeometry,TorusGeometry:BufferGeometry,BoxGeometry:BufferGeometry,TubeGeometry:class extends BufferGeometry{constructor(c){super(); for(let i=0;i<=10;i++) c.getPoint(i/10, new Vector3())}},
 sRGBEncoding:3001};
