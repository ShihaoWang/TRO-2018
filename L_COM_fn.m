function L_COM = L_COM_fn(State, p)

mBody = p.mBody;

mLeftShoulder = p.mLeftShoulder;            mLeftArm = p.mLeftArm;              mLeftHand = p.mLeftHand;

mRightShoulder = p.mRightShoulder;          mRightArm = p.mRightArm;            mRightHand = p.mRightHand;

mLeftHip = p.mLeftHip;                      mLeftThigh = p.mLeftThigh;          mLeftShank = p.mLeftShank;          mLeftAnkle = p.mLeftAnkle;
mLeftFoot = p.mLeftFoot;

mRightHip = p.mRightHip;                    mRightThigh = p.mRightThigh;        mRightShank = p.mRightShank;        mRightAnkle = p.mRightAnkle;
mRightFoot = p.mRightFoot;

%% Retrieve the value from the state variables
Vel = Rigid_Body_Velocity( State, p );

vBody = Vel.vBody;

vLeftShoulder = Vel.vLeftShoulder;              vLeftArm = Vel.vLeftArm;                vLeftHand = Vel.vLeftHand;

vRightShoulder = Vel.vRightShoulder;            vRightArm = Vel.vRightArm;              vRightHand = Vel.vRightHand;

vLeftHip = Vel.vLeftHip;            vLeftThigh = Vel.vLeftThigh;            vLeftShank = Vel.vLeftShank;            vLeftAnkle = Vel.vLeftAnkle;
vLeftFoot = Vel.vLeftFoot;

vRightHip = Vel.vRightHip;          vRightThigh = Vel.vRightThigh;          vRightShank = Vel.vRightShank;          vRightAnkle = Vel.vRightAnkle;
vRightFoot = Vel.vRightFoot;


L_COM = mBody * vBody + mLeftShoulder * vLeftShoulder + mLeftArm * vLeftArm + mLeftHand * vLeftHand + ...
                        mRightShoulder * vRightShoulder + mRightArm * vRightArm + mRightHand * vRightHand + ...
                        mLeftHip * vLeftHip + mLeftThigh * vLeftThigh + mLeftShank * vLeftShank + mLeftAnkle * vLeftAnkle + mLeftFoot * vLeftFoot + ...
                        mRightHip * vRightHip + mRightThigh * vRightThigh + mRightShank * vRightShank + mRightAnkle * vRightAnkle + mRightFoot * vRightFoot;

end

