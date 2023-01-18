/*
 * Global variables
 */
var meshResolution;

// Particle states
var mass;
var vertexPosition, vertexNormal;
var vertexVelocity;

// Spring properties
var K, restLength; 

// Force parameters
var Cd;
var uf, Cv;


/*
 * Getters and setters
 */
function getPosition(i, j) {
    var id = i*meshResolution + j;
    return vec3.create([vertexPosition[3*id], vertexPosition[3*id + 1], vertexPosition[3*id + 2]]);
}

function setPosition(i, j, x) {
    var id = i*meshResolution + j;
    vertexPosition[3*id] = x[0]; vertexPosition[3*id + 1] = x[1]; vertexPosition[3*id + 2] = x[2];
}

function getNormal(i, j) {
    var id = i*meshResolution + j;
    return vec3.create([vertexNormal[3*id], vertexNormal[3*id + 1], vertexNormal[3*id + 2]]);
}

function getVelocity(i, j) {
    var id = i*meshResolution + j;
    return vec3.create(vertexVelocity[id]);
}

function setVelocity(i, j, v) {
    var id = i*meshResolution + j;
    vertexVelocity[id] = vec3.create(v);
}


/*
 * Provided global functions (you do NOT have to modify them)
 */
function computeNormals() {
    var dx = [1, 1, 0, -1, -1, 0], dy = [0, 1, 1, 0, -1, -1];
    var e1, e2;
    var i, j, k = 0, t;
    for ( i = 0; i < meshResolution; ++i )
        for ( j = 0; j < meshResolution; ++j ) {
            var p0 = getPosition(i, j), norms = [];
            for ( t = 0; t < 6; ++t ) {
                var i1 = i + dy[t], j1 = j + dx[t];
                var i2 = i + dy[(t + 1) % 6], j2 = j + dx[(t + 1) % 6];
                if ( i1 >= 0 && i1 < meshResolution && j1 >= 0 && j1 < meshResolution &&
                     i2 >= 0 && i2 < meshResolution && j2 >= 0 && j2 < meshResolution ) {
                    e1 = vec3.subtract(getPosition(i1, j1), p0);
                    e2 = vec3.subtract(getPosition(i2, j2), p0);
                    norms.push(vec3.normalize(vec3.cross(e1, e2)));
                }
            }
            e1 = vec3.create();
            for ( t = 0; t < norms.length; ++t ) vec3.add(e1, norms[t]);
            vec3.normalize(e1);
            vertexNormal[3*k] = e1[0];
            vertexNormal[3*k + 1] = e1[1];
            vertexNormal[3*k + 2] = e1[2];
            ++k;
        }
}


var clothIndex, clothWireIndex;
function initMesh() {
    var i, j, k;

    vertexPosition = new Array(meshResolution*meshResolution*3);
    vertexNormal = new Array(meshResolution*meshResolution*3);
    clothIndex = new Array((meshResolution - 1)*(meshResolution - 1)*6);
    clothWireIndex = [];

    vertexVelocity = new Array(meshResolution*meshResolution);
    restLength[0] = 4.0/(meshResolution - 1);
    restLength[1] = Math.sqrt(2.0)*4.0/(meshResolution - 1);
    restLength[2] = 2.0*restLength[0];

    for ( i = 0; i < meshResolution; ++i )
        for ( j = 0; j < meshResolution; ++j ) {
            setPosition(i, j, [-2.0 + 4.0*j/(meshResolution - 1), -2.0 + 4.0*i/(meshResolution - 1), 0.0]);
            setVelocity(i, j, vec3.create());

            if ( j < meshResolution - 1 )
                clothWireIndex.push(i*meshResolution + j, i*meshResolution + j + 1);
            if ( i < meshResolution - 1 )
                clothWireIndex.push(i*meshResolution + j, (i + 1)*meshResolution + j);
            if ( i < meshResolution - 1 && j < meshResolution - 1 )
                clothWireIndex.push(i*meshResolution + j, (i + 1)*meshResolution + j + 1);
        }
    computeNormals();

    k = 0;
    for ( i = 0; i < meshResolution - 1; ++i )
        for ( j = 0; j < meshResolution - 1; ++j ) {
            clothIndex[6*k] = i*meshResolution + j;
            clothIndex[6*k + 1] = i*meshResolution + j + 1;
            clothIndex[6*k + 2] = (i + 1)*meshResolution + j + 1;
            clothIndex[6*k + 3] = i*meshResolution + j;
            clothIndex[6*k + 4] = (i + 1)*meshResolution + j + 1;            
            clothIndex[6*k + 5] = (i + 1)*meshResolution + j;
            ++k;
        }
}


/*
 * KEY function: simulate one time-step using Euler's method
 */
function simulate(stepSize) {
    // FIX ME
    const i_offset = [0, -1, 0, 1, 1, 1, -1, -1, 0, -2, 0, 2];
    const j_offset = [-1, 0, 1, 0, 1, -1, 1, -1, -2, 0, 2, 0]
    let q_i = 0;
    let q_j = 0;
    let spring_force = vec3.create([0, 0, 0]);
    const gravity_force = vec3.create([0, -9.8, 0]);
    let damping_force;
    let fluid_force;
    // let rest_lengths = [4/(meshResolution - 1), 4 * Math.sqrt(2)/(meshResolution - 1), 8/(meshResolution - 1)];

    let tmp_pos = {}
    for (let i = 0; i < meshResolution; i++) {
        for (let j = 0; j < meshResolution; j++) {
            // 1. spring forces: 12 points
            spring_force.set([0, 0, 0])
            let p = getPosition(i, j);
            for (let n = 0; n < i_offset.length; n++) {
                q_i = i + i_offset[n];
                q_j = j + j_offset[n];
                if (q_i < 0 || q_j < 0 || q_i >= meshResolution || q_j >= meshResolution) {
                    continue;
                }
                let q = getPosition(q_i, q_j);

                // 1.1 spring force
                let res_idx = Math.floor(n / 4);
                let cur_rest_length = restLength[res_idx];
                let cur_k = K[res_idx];
                let length = vec3.length(vec3.subtract(vec3.create(p), q));
                let cur_spring_force = vec3.scale(vec3.subtract(vec3.create(p), q), cur_k * (cur_rest_length - length) / length);
                vec3.add(spring_force, cur_spring_force);
            }

            // 1.2 gravity force, already have

            // 1.3 damping
            damping_force = vec3.scale(getVelocity(i, j), -Cd);

            // 1.4 viscous fluid
            fluid_force = vec3.scale( getNormal(i, j), vec3.dot( getNormal(i, j), vec3.subtract(vec3.create(uf), getVelocity(i, j)) ) * Cv);

            // 1.5 total force
            let acc_force = vec3.add( vec3.add( vec3.add(vec3.create(spring_force), gravity_force), damping_force), fluid_force);

            // 2. update velocity
            let new_v = vec3.add(getVelocity(i, j), vec3.scale(acc_force, stepSize / mass));
            setVelocity(i, j, new_v);

            if (i === meshResolution - 1 && (j === 0 || j === meshResolution - 1) ) {
                continue;
            }
            let new_p = vec3.add(getPosition(i, j), vec3.scale(new_v, stepSize));

            if (!(i in tmp_pos)) {
                tmp_pos[i] = {};
            }
            tmp_pos[i][j] = new_p;
        }
    }

    for (let i = 0; i < meshResolution; i++) {
        for (let j = 0; j < meshResolution; j++) {
            if (i === meshResolution - 1 && (j === 0 || j === meshResolution - 1) ) {
                continue;
            }
            setPosition(i, j, tmp_pos[i][j]);
        }
    }
}
