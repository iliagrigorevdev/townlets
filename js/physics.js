
const MAX_SUB_STEPS = 10;
const FIXED_TIME_STEP = 0.005;
const TIME_STEP_MAX = MAX_SUB_STEPS * FIXED_TIME_STEP;

const GRAVITY = 9.81;

const GROUP_GROUND = 0x01;
const GROUP_TOWNLET = 0x02;
const GROUP_BAT = 0x04;

const MASK_STATIC = GROUP_GROUND;
const MASK_DYNAMIC = GROUP_BAT | GROUP_TOWNLET;
const MASK_ALL = MASK_STATIC | MASK_DYNAMIC;

const GROUND_HALF_DEPTH = 0.5;
const GROUND_HALF_SIZE = 50.0;

const BAT_LENGTH = 0.9;
const BAT_HALF_LENGTH = 0.5 * BAT_LENGTH;
const BAT_LENGTH_SQUARED = BAT_LENGTH * BAT_LENGTH;
const BAT_DIAMETER = 0.035;
const BAT_RADIUS = 0.5 * BAT_DIAMETER;
const BAT_RADIUS_SQUARED = BAT_RADIUS * BAT_RADIUS;
const BAT_VOLUME = Math.PI * BAT_RADIUS_SQUARED * BAT_LENGTH;
const BAT_DENSITY = 1100.0;
const BAT_MASS = BAT_VOLUME * BAT_DENSITY;
const BAT_I11 = BAT_MASS * (BAT_RADIUS_SQUARED / 4.0 + BAT_LENGTH_SQUARED / 12.0);
const BAT_I22 = BAT_I11;
const BAT_I33 = BAT_MASS * BAT_RADIUS_SQUARED / 2.0;

const TOWNLET_LENGTH = 0.2;
const TOWNLET_HALF_LENGTH = 0.5 * TOWNLET_LENGTH;
const TOWNLET_LENGTH_SQUARED = TOWNLET_LENGTH * TOWNLET_LENGTH;
const TOWNLET_DIAMETER = 0.05;
const TOWNLET_RADIUS = 0.5 * TOWNLET_DIAMETER;
const TOWNLET_RADIUS_SQUARED = TOWNLET_RADIUS * TOWNLET_RADIUS;
const TOWNLET_VOLUME = Math.PI * TOWNLET_RADIUS_SQUARED * TOWNLET_LENGTH;
const TOWNLET_DENSITY = 750.0;
const TOWNLET_MASS = TOWNLET_VOLUME * TOWNLET_DENSITY;
const TOWNLET_I11 = TOWNLET_MASS * (TOWNLET_RADIUS_SQUARED / 4.0 + TOWNLET_LENGTH_SQUARED / 12.0);
const TOWNLET_I22 = TOWNLET_MASS * TOWNLET_RADIUS_SQUARED / 2.0;
const TOWNLET_I33 = TOWNLET_I11;

const SHOT_HEIGHT = 1.2;
const SHOT_DISTANCE = 6.0;
const SHOT_ANGLE_MAX = Math.PI / 18.0;
const SHOT_ANGLE_MIN = -SHOT_ANGLE_MAX;
const SHOT_VELOCITY_MIN = 4.0;
const SHOT_VELOCITY_MAX = 9.0;
const SHOT_SPIN_MIN = Math.PI;
const SHOT_SPIN_MAX = 4.0 * Math.PI;
const SHOT_ELEVATION = Math.PI / 9.0;

const FRICTION_GROUND = 0.7;
const FRICTION_BAT = 0.5;
const FRICTION_TOWNLET = 0.5;
const ROLLING_FRICTION_GROUND = 0.1;
const ROLLING_FRICTION_BAT = 0.1;
const ROLLING_FRICTION_TOWNLET = 0.1;
const RESTITUTION_GROUND = 0.3;
const RESTITUTION_BAT = 0.5;
const RESTITUTION_TOWNLET = 0.5;

const TOWNLET_FIGURES = [
	[ // Row
		[ 0.0, TOWNLET_HALF_LENGTH, 0.0, 0.0, 0.0, 0.0 ],
		[ 2.0 * TOWNLET_DIAMETER, TOWNLET_HALF_LENGTH, 0.0, 0.0, 0.0, 0.0 ],
		[ -2.0 * TOWNLET_DIAMETER, TOWNLET_HALF_LENGTH, 0.0, 0.0, 0.0, 0.0 ],
		[ 4.0 * TOWNLET_DIAMETER, TOWNLET_HALF_LENGTH, 0.0, 0.0, 0.0, 0.0 ],
		[ -4.0 * TOWNLET_DIAMETER, TOWNLET_HALF_LENGTH, 0.0, 0.0, 0.0, 0.0 ],
	],
	[ // Star
		[ 0.0, TOWNLET_HALF_LENGTH, 0.0, 0.0, 0.0, 0.0 ],
		[ 0.0, TOWNLET_RADIUS, TOWNLET_HALF_LENGTH + TOWNLET_RADIUS, 90.0, 0.0, 0.0 ],
		[ 0.0, TOWNLET_RADIUS, -TOWNLET_HALF_LENGTH - TOWNLET_RADIUS, 90.0, 0.0, 0.0 ],
		[ TOWNLET_HALF_LENGTH + TOWNLET_RADIUS, TOWNLET_RADIUS, 0.0, 0.0, 0.0, 90.0 ],
		[ -TOWNLET_HALF_LENGTH - TOWNLET_RADIUS, TOWNLET_RADIUS, 0.0, 0.0, 0.0, 90.0 ],
	]
];
const TOWNLET_FIGURES_COUNT = TOWNLET_FIGURES.length;

class Physics {
	constructor() {
		this.simulationActive = false;
		this.simulationTime = 0.0;

		this.collisionConfiguration = new Ammo.btDefaultCollisionConfiguration();
		this.dispatcher = new Ammo.btCollisionDispatcher(this.collisionConfiguration);
		this.overlappingPairCache = new Ammo.btDbvtBroadphase();
		this.solver = new Ammo.btSequentialImpulseConstraintSolver();
		this.dynamicsWorld = new Ammo.btDiscreteDynamicsWorld(this.dispatcher,
				this.overlappingPairCache, this.solver, this.collisionConfiguration);

		this.dynamicsWorld.setGravity(new Ammo.btVector3(0.0, -GRAVITY, 0.0));

		var groundShape = new Ammo.btBoxShape(new Ammo.btVector3(
				GROUND_HALF_SIZE, GROUND_HALF_DEPTH, GROUND_HALF_SIZE));
		this.createStaticBody(groundShape, 0.0, -GROUND_HALF_DEPTH, 0.0, 0.0, 0.0, 0.0, 1.0,
				GROUP_GROUND, MASK_DYNAMIC, FRICTION_GROUND, ROLLING_FRICTION_GROUND, RESTITUTION_GROUND);

		this.reset();
	}

	update(dt) {
		if (this.simulationActive) {
			this.simulationTime += Math.min(dt, TIME_STEP_MAX);

			this.dynamicsWorld.stepSimulation(dt, MAX_SUB_STEPS, FIXED_TIME_STEP);
		}
	}

	createBody(collisionShape, mass, I11, I22, I33, x, y, z, qx, qy, qz, qw,
			group, mask, friction, rollingFriction, restitution) {
		var transform = new Ammo.btTransform();
		transform.setIdentity();
		transform.setOrigin(new Ammo.btVector3(x, y, z));
		transform.setRotation(new Ammo.btQuaternion(qx, qy, qz, qw));
		var motionState = new Ammo.btDefaultMotionState(transform);
		var constructionInfo = new Ammo.btRigidBodyConstructionInfo(mass, motionState,
				collisionShape, new Ammo.btVector3(I11, I22, I33));
		constructionInfo.set_m_friction(friction);
		constructionInfo.set_m_rollingFriction(rollingFriction);
		constructionInfo.set_m_restitution(restitution);
		var body = new Ammo.btRigidBody(constructionInfo);
		this.dynamicsWorld.addRigidBody(body, group, mask);
		return body;
	}

	destroyBody(body) {
		this.dynamicsWorld.removeRigidBody(body);
	}

	getBatShape() {
		if (!this.batShape) {
			this.batShape = new Ammo.btCylinderShapeZ(new Ammo.btVector3(
					BAT_RADIUS, BAT_RADIUS, BAT_HALF_LENGTH));
		}
		return this.batShape;
	}

	getTownletShape() {
		if (!this.townletShape) {
			this.townletShape = new Ammo.btCylinderShape(new Ammo.btVector3(
					TOWNLET_RADIUS, TOWNLET_HALF_LENGTH, TOWNLET_RADIUS));
		}
		return this.townletShape;
	}

	createStaticBody(collisionShape, x, y, z, qx, qy, qz, qw,
			group, mask, friction, rollingFriction, restitution) {
		var staticBody = this.createBody(collisionShape, 0.0, 0.0, 0.0, 0.0,
				x, y, z, qx, qy, qz, qw, group, mask, friction, rollingFriction, restitution);
		return staticBody;
	}

	createBatBody(x, y, z) {
		var batBody = this.createBody(this.getBatShape(), BAT_MASS,
				BAT_I11, BAT_I22, BAT_I33, x, y, z, 0.0, 0.0, 0.0, 1.0,
				GROUP_BAT, MASK_ALL, FRICTION_BAT, ROLLING_FRICTION_BAT, RESTITUTION_BAT);
		return batBody;
	}

	createTownletBody(x, y, z, qx, qy, qz, qw) {
		var townletBody = this.createBody(this.getTownletShape(), TOWNLET_MASS,
				TOWNLET_I11, TOWNLET_I22, TOWNLET_I33, x, y, z, qx, qy, qz, qw,
				GROUP_TOWNLET, MASK_ALL, FRICTION_TOWNLET, ROLLING_FRICTION_TOWNLET, RESTITUTION_TOWNLET);
		return townletBody;
	}

	reset(hard = true, figureIndex = 0) {
		this.currentFigureIndex = figureIndex;

		if (this.batBody) {
			this.dynamicsWorld.removeRigidBody(this.batBody);
		}
		this.batBody = this.createBatBody(0.0, SHOT_HEIGHT, SHOT_DISTANCE);

		if (hard) {
			if (this.townletBodies) {
				for (var i = 0; i < this.townletBodies.length; i++) {
					this.dynamicsWorld.removeRigidBody(this.townletBodies[i]);
				}
			}
			var townletFigure = TOWNLET_FIGURES[figureIndex];
			this.townletBodies = new Array(townletFigure.length);
			for (var i = 0; i < this.townletBodies.length; i++) {
				var t = townletFigure[i];
				var q = new Ammo.btQuaternion(0.0, 0.0, 0.0, 1.0);
				q.setEulerZYX(t[5] / 180.0 * Math.PI, t[4] / 180.0 * Math.PI, t[3] / 180.0 * Math.PI);
				this.townletBodies[i] = this.createTownletBody(t[0], t[1], t[2], q.x(), q.y(), q.z(), q.w());
			}
		}

		this.simulationActive = false;
		this.simulationTime = 0.0;
	}

	shot(angle, velocity, spin) {
		var shotAngle = Math.max(SHOT_ANGLE_MIN, Math.min(SHOT_ANGLE_MAX, angle));
		var shotVelocity = Math.max(SHOT_VELOCITY_MIN, Math.min(SHOT_VELOCITY_MAX, velocity));
		var shotSpin = Math.max(SHOT_SPIN_MIN, Math.min(SHOT_SPIN_MAX, spin));
		var linearVelocity = new Ammo.btVector3(0.0, 0.0, -shotVelocity)
				.rotate(new Ammo.btVector3(0, 1, 0), shotAngle)
				.rotate(new Ammo.btVector3(1, 0, 0), SHOT_ELEVATION);
		var angularVelocity = new Ammo.btVector3(0.0, shotSpin, 0.0);
		this.batBody.setLinearVelocity(linearVelocity);
		this.batBody.setAngularVelocity(angularVelocity);
		this.simulationActive = true;
	}
}
