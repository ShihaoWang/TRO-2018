function I_COM = I_COM_fn(State,p)

% This function is used to calculate the moment of inertia with respect to the center of mass

% This is to compute the angular moment of the robot with respect to the
% center of mass of the robot

%% Individual mass
mBody = p.mBody;

mLeftShoulder = p.mLeftShoulder;            mLeftArm = p.mLeftArm;              mLeftHand = p.mLeftHand;

mRightShoulder = p.mRightShoulder;          mRightArm = p.mRightArm;            mRightHand = p.mRightHand;

mLeftHip = p.mLeftHip;                      mLeftThigh = p.mLeftThigh;          mLeftShank = p.mLeftShank;          mLeftAnkle = p.mLeftAnkle;
mLeftFoot = p.mLeftFoot;

mRightHip = p.mRightHip;                    mRightThigh = p.mRightThigh;        mRightShank = p.mRightShank;        mRightAnkle = p.mRightAnkle;
mRightFoot = p.mRightFoot;

%% Individual moment of inertia
IBody = p.IBody;

ILeftShoulder = p.ILeftShoulder;            ILeftArm = p.ILeftArm;              ILeftHand = p.ILeftHand;

IRightShoulder = p.IRightShoulder;          IRightArm = p.IRightArm;            IRightHand = p.IRightHand;

ILeftHip = p.ILeftHip;                      ILeftThigh = p.ILeftThigh;          ILeftShank = p.ILeftShank;          ILeftAnkle = p.ILeftAnkle;
ILeftFoot = p.ILeftFoot;

IRightHip = p.IRightHip;                    IRightThigh = p.IRightThigh;        IRightShank = p.IRightShank;        IRightAnkle = p.IRightAnkle;
IRightFoot = p.IRightFoot;

%% Position
Pos = Rigid_Body_Position( State, p );
rBody = Pos.rBody;

rLeftShoulder = Pos.rLeftShoulder;              rLeftArm = Pos.rLeftArm;                rLeftHand = Pos.rLeftHand;

rRightShoulder = Pos.rRightShoulder;            rRightArm = Pos.rRightArm;              rRightHand = Pos.rRightHand;

rLeftHip = Pos.rLeftHip;            rLeftThigh = Pos.rLeftThigh;            rLeftShank = Pos.rLeftShank;
rLeftAnkle = Pos.rLeftAnkle;        rLeftFoot = Pos.rLeftFoot;

rRightHip = Pos.rRightHip;          rRightThigh = Pos.rRightThigh;          rRightShank = Pos.rRightShank;
rRightAnkle = Pos.rRightAnkle;      rRightFoot = Pos.rRightFoot;

rCOM = Pos.rCOM;

%% Rotation
Rot = Rigid_Body_Rotation( State, p );
RBody = Rot.RotBody;

RLeftShoulder = Rot.RotLeftShoulder;            RLeftArm = Rot.RotLeftArm;          RLeftHand = Rot.RotLeftHand;

RRightShoulder = Rot.RotRightShoulder;          RRightArm = Rot.RotRightArm;        RRightHand = Rot.RotRightHand;

RLeftHip = Rot.RotLeftHip;              RLeftThigh = Rot.RotLeftThigh;              RLeftShank = Rot.RotLeftShank;
RLeftAnkle = Rot.RotLeftAnkle;          RLeftFoot = Rot.RotLeftFoot;

RRightHip = Rot.RotRightHip;            RRightThigh = Rot.RotRightThigh;            RRightShank = Rot.RotRightShank;
RRightAnkle = Rot.RotRightAnkle;        RRightFoot = Rot.RotRightFoot;

%% Moment of Inertia

I_Body2COM = mBody * diag([rBody - rCOM]) * diag([rBody - rCOM]) + RBody * IBody * RBody';

I_LeftShoulder2COM = mLeftShoulder * diag([rLeftShoulder - rCOM]) * diag([rLeftShoulder - rCOM]) + RLeftShoulder * ILeftShoulder * RLeftShoulder';
I_LeftArm2COM = mLeftArm * diag([rLeftArm - rCOM]) * diag([rLeftArm - rCOM]) + RLeftArm * ILeftArm * RLeftArm';
I_LeftHand2COM = mLeftHand * diag([rLeftHand - rCOM]) * diag([rLeftHand - rCOM]) + RLeftHand * ILeftHand * RLeftHand';

I_RightShoulder2COM = mRightShoulder * diag([rRightShoulder - rCOM]) * diag([rRightShoulder - rCOM]) + RRightShoulder * IRightShoulder * RRightShoulder';
I_RightArm2COM = mRightArm * diag([rRightArm - rCOM]) * diag([rRightArm - rCOM]) + RRightArm * IRightArm * RRightArm';
I_RightHand2COM = mRightHand * diag([rRightHand - rCOM]) * diag([rRightHand - rCOM]) + RRightHand * IRightArm * RRightHand';

I_LeftHip2COM = mLeftHip * diag([rLeftHip - rCOM]) * diag([rLeftHip - rCOM]) + RLeftHip * ILeftHip * RLeftHip';
I_LeftThigh2COM = mLeftThigh * diag([rLeftThigh - rCOM]) * diag([rLeftThigh - rCOM]) + RLeftThigh * ILeftThigh * RLeftThigh';
I_LeftShank2COM = mLeftShank * diag([rLeftShank - rCOM]) * diag([rLeftShank - rCOM]) + RLeftShank * ILeftShank * RLeftShank';
I_LeftAnkle2COM = mLeftAnkle * diag([rLeftAnkle - rCOM]) * diag([rLeftAnkle - rCOM]) + RLeftAnkle * ILeftAnkle * RLeftAnkle';
I_LeftFoot2COM = mLeftFoot * diag([rLeftFoot - rCOM]) * diag([rLeftFoot - rCOM]) + RLeftFoot * ILeftFoot * RLeftFoot';

I_RightHip2COM = mRightHip * diag([rRightHip - rCOM]) * diag([rRightHip - rCOM]) + RRightHip * IRightHip * RRightHip';
I_RightThigh2COM = mRightThigh * diag([rRightThigh - rCOM]) * diag([rRightThigh - rCOM]) + RRightThigh * IRightThigh * RRightThigh';
I_RightShank2COM = mRightShank * diag([rRightShank - rCOM]) * diag([rRightShank - rCOM]) + RRightShank * IRightShank * RRightShank';
I_RightAnkle2COM = mRightAnkle * diag([rRightAnkle - rCOM]) * diag([rRightAnkle - rCOM]) + RRightAnkle * IRightAnkle * RRightAnkle';
I_RightFoot2COM = mRightFoot * diag([rRightFoot - rCOM]) * diag([rRightFoot - rCOM]) + RRightFoot * IRightFoot * RRightFoot';

I_COM = I_Body2COM + I_LeftShoulder2COM + I_LeftArm2COM + I_LeftHand2COM + I_RightShoulder2COM + I_RightArm2COM + I_RightHand2COM + ...
        I_LeftHip2COM + I_LeftThigh2COM + I_LeftShank2COM + I_LeftAnkle2COM + I_LeftFoot2COM + ...
        I_RightHip2COM + I_RightThigh2COM + I_RightShank2COM + I_RightAnkle2COM + I_RightFoot2COM;
    
%% Now this is the moment of inertia computed at the center of mass of the robot

end


