function H_COM = H_COM_fn(State, p)

% This is to compute the angular moment of the robot with respect to the
% center of mass of the robot

mBody = p.mBody;

mLeftShoulder = p.mLeftShoulder;            mLeftArm = p.mLeftArm;              mLeftHand = p.mLeftHand;

mRightShoulder = p.mRightShoulder;          mRightArm = p.mRightArm;            mRightHand = p.mRightHand;

mLeftHip = p.mLeftHip;                      mLeftThigh = p.mLeftThigh;          mLeftShank = p.mLeftShank;          mLeftAnkle = p.mLeftAnkle;
mLeftFoot = p.mLeftFoot;

mRightHip = p.mRightHip;                    mRightThigh = p.mRightThigh;        mRightShank = p.mRightShank;        mRightAnkle = p.mRightAnkle;
mRightFoot = p.mRightFoot;

%% Position
Pos = Rigid_Body_Position( State, p );
rBody = Pos.rBody;

rLeftShoulder = Pos.rLeftShoulder;              rLeftArm = Pos.rLeftArm;                rLeftHand = Pos.rLeftHand;

rRightShoulder = Pos.rRightShoulder;            rRightArm = Pos.rRightArm;              rRightHand = Pos.rRightHand;

rLeftHip = Pos.rLeftHip;            rLeftThigh = Pos.rLeftThigh;            rLeftShank = Pos.rLeftShank;
rLeftAnkle = Pos.rLeftAnkle;        rLeftFoot = Pos.rLeftFoot;

rRightHip = Pos.rRightHip;          rRightThigh = Pos.rRightThigh;          rRightShank = Pos.rRightShank;
rRightAnkle = Pos.rRightAnkle;      rRightFoot = Pos.rRightFoot;

%% Velocity
Vel = Rigid_Body_Velocity( State, p );

vBody = Vel.vBody;

vLeftShoulder = Vel.vLeftShoulder;              vLeftArm = Vel.vLeftArm;                vLeftHand = Vel.vLeftHand;

vRightShoulder = Vel.vRightShoulder;            vRightArm = Vel.vRightArm;              vRightHand = Vel.vRightHand;

vLeftHip = Vel.vLeftHip;            vLeftThigh = Vel.vLeftThigh;            vLeftShank = Vel.vLeftShank;            vLeftAnkle = Vel.vLeftAnkle;
vLeftFoot = Vel.vLeftFoot;

vRightHip = Vel.vRightHip;          vRightThigh = Vel.vRightThigh;          vRightShank = Vel.vRightShank;          vRightAnkle = Vel.vRightAnkle;
vRightFoot = Vel.vRightFoot;

H_COM_Trans =   cross(rBody - rCOM, mBody * vBody) + cross(rLeftShoulder - rCOM, mLeftShoulder * vLeftShoulder) + cross(rLeftArm - rCOM, mLeftArm * vLeftArm) + ...
                cross(rLeftHand - rCOM, mLeftHand * vLeftHand) + cross(rRightShoulder - rCOM, mRightShoulder * vRightShoulder) + cross(rRightArm - rCOM, mRightArm * vRightArm) + ...
                cross(rRightHand - rCOM, mRightHand * vRightHand) + cross(rLeftHip - rCOM, mLeftHip * vLeftHip) + cross(rLeftThigh - rCOM, mLeftThigh * vLeftThigh) + ...
                cross(rLeftShank - rCOM, mLeftShank * vLeftShank) + cross(rLeftAnkle - rCOM, mLeftAnkle * vLeftAnkle) + cross(rLeftFoot - rCOM, mLeftFoot * vLeftFoot) + ...
                cross(rRightHip - rCOM, mRightHip * vRightHip) + cross(rRightThigh - rCOM, mRightThigh * vRightThigh) + cross(rRightShank - rCOM, mRightShank * vRightShank) + ...
                cross(rRightAnkle - rCOM, mRightAnkle * vRightAnkle) + cross(rRightFoot - rCOM, mRightFoot * vRightFoot);

%% The next part of the angular momentum is the rotational part
IBody = p.IBody;

ILeftShoulder = p.ILeftShoulder;            ILeftArm = p.ILeftArm;              ILeftHand = p.ILeftHand = ILeftHand;

IRightShoulder = p.IRightShoulder;          IRightArm = p.IRightArm;            IRightHand = p.IRightHand;

ILeftHip = p.ILeftHip;                      ILeftThigh = p.ILeftThigh;          ILeftShank = p.ILeftShank;          ILeftAnkle = p.ILeftAnkle;
ILeftFoot = p.ILeftFoot;

IRightHip = p.IRightHip;                    IRightThigh = p.IRightThigh;        IRightShank = p.IRightShank;        IRightAnkle = p.IRightAnkle;
IRightFoot = p.IRightFoot;

%% However, the momentum of inertia tensor needs to be transformed into the reference frame

%% Rotation matrix
Rot = Rigid_Body_Rotation( State, p );
RBody = Rot.RotBody;

RLeftShoulder = Rot.RotLeftShoulder;            RLeftArm = Rot.RotLeftArm;          RLeftHand = Rot.RotLeftHand;

RRightShoulder = Rot.RotRightShoulder;          RRightArm = Rot.RotRightArm;        RRightHand = Rot.RotRightHand;

RLeftHip = Rot.RotLeftHip;              RLeftThigh = Rot.RotLeftThigh;              RLeftShank = Rot.RotLeftShank;
RLeftAnkle = Rot.RotLeftAnkle;          RLeftFoot = Rot.RotLeftFoot;

RRightHip = Rot.RotRightHip;            RRightThigh = Rot.RotRightThigh;            RRightShank = Rot.RotRightShank;
RRightAnkle = Rot.RotRightAnkle;        RRightFoot = Rot.RotRightFoot;

%% Angular velocity
omega = Rigid_Body_Omega( State, p );

omegaBody = omega.omegaBody;

omegaLeftShoulder = omega.omegaLeftShoulder;            omegaLeftArm = omega.omegaLeftArm;          omegaLeftHand = omega.omegaLeftHand;
omegaRightShoulder = omega.omegaRightShoulder;          omegaRightArm = omega.omegaRightArm;        omegaRightHand = omega.omegaRightHand;

omegaLeftHip = omega.omegaLeftHip;          omegaLeftThigh = omega.omegaLeftThigh;          omegaLeftShank = omega.omegaLeftShank;
omegaLeftAnkle = omega.omegaLeftAnkle;      omegaLeftFoot = omega.omegaLeftFoot;

omegaRightHip = omega.omegaRightHip;        omegaRightThigh = omega.omegaRightThigh;        omegaRightShank = omega.omegaRightShank;
omegaRightAnkle = omega.omegaRightAnkle;    omegaRightFoot = omega.omegaRightFoot;

H_COM_Rot = RBody * IBody * omegaBody + RLeftShoulder * ILeftShoulder * omegaLeftShoulder + RLeftArm * ILeftArm * omegaLeftArm + ...
            RLeftHand * ILeftHand * omegaLeftHand + RRightShoulder * IRightShoulder * omegaRightShoulder + RRightArm * IRightArm * omegaRightArm + ...
            RRightHand * IRightHand * omegaRightHand + RLeftHip * ILeftHip * omegaLeftHip + RLeftThigh * ILeftThigh * omegaLeftThigh + ...
            RLeftShank * ILeftShank * omegaLeftShank + RLeftAnkle * ILeftAnkle * omegaLeftAnkle + RLeftFoot * ILeftFoot * omegaLeftFoot + ...
            RRightHip * IRightHip * omegaRightHip + RRightThigh * IRightThigh * omegaRightThigh + RRightShank * IRightShank * omegaRightShank + ...
            RRightAnkle * IRightAnkle * omegaRightAnkle + RRightFoot * IRightFoot * omegaRightFoot;

H_COM = H_COM_Trans + H_COM_Rot;

end

