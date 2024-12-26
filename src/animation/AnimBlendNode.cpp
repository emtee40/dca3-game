#include "common.h"

#include "AnimBlendAssociation.h"
#include "AnimBlendNode.h"

void 
CAnimBlendNode::Init(void)
{
	frameA = -1;
	frameB = -1;
	remainingTime = 0.0f;
	sequence = nil;
	association = nil;
}

bool
CAnimBlendNode::Update(CVector &trans, CQuaternion &rot, float weight)
{
	bool looped = false;

	trans = CVector(0.0f, 0.0f, 0.0f);
	rot = CQuaternion(0.0f, 0.0f, 0.0f, 0.0f);

	if(association->IsRunning()){
		remainingTime -= association->timeStep;
		if(remainingTime <= 0.0f)
			looped = NextKeyFrame();
	}

	float blend = association->GetBlendAmount(weight);
	if(blend > 0.0f){
		float kfAdt = sequence->GetDeltaTime(frameA);
		float t = kfAdt == 0.0f ? 0.0f : (kfAdt - remainingTime)/kfAdt;
		if(sequence->type & CAnimBlendSequence::KF_TRANS){
			auto kfAt = sequence->GetTranslation(frameA);
			auto kfBt = sequence->GetTranslation(frameB);
			trans = kfBt + t*(kfAt - kfBt);
			trans *= blend;
		}
		if(sequence->type & CAnimBlendSequence::KF_ROT){
			auto kfAr = sequence->GetRotation(frameA);
			auto kfBr = sequence->GetRotation(frameB);
			rot.Slerp(kfBr, kfAr, theta, invSin, t);
			rot *= blend;
		}
	}

	return looped;
}

bool
CAnimBlendNode::NextKeyFrame(void)
{
	bool looped;

	if(sequence->numFrames <= 1)
		return false;

	looped = false;
	frameB = frameA;

	// Advance as long as we have to
	while(remainingTime <= 0.0f){
		frameA++;

		if(frameA >= sequence->numFrames){
			// reached end of animation
			if(!association->IsRepeating()){
				frameA--;
				remainingTime = 0.0f;
				return false;
			}
			looped = true;
			frameA = 0;
		}

		remainingTime += sequence->GetDeltaTime(frameA);
	}

	frameB = frameA - 1;
	if(frameB < 0)
		frameB += sequence->numFrames;

	CalcDeltas();
	return looped;
}

// Set animation to time t
bool
CAnimBlendNode::FindKeyFrame(float t)
{
	if(sequence->numFrames < 1)
		return false;

	frameA = 0;
	frameB = frameA;

	if(sequence->numFrames >= 2){
		frameA++;

		// advance until t is between frameB and frameA
		while(t > sequence->GetDeltaTime(frameA)){
			t -= sequence->GetDeltaTime(frameA);
			frameB = frameA++;
			if(frameA >= sequence->numFrames){
				// reached end of animation
				if(!association->IsRepeating())
					return false;
				frameA = 0;
				frameB = 0;
			}
		}

		remainingTime = sequence->GetDeltaTime(frameA) - t;
	}

	CalcDeltas();
	return true;
}

void
CAnimBlendNode::CalcDeltas(void)
{
	if((sequence->type & CAnimBlendSequence::KF_ROT) == 0)
		return;
	auto kfAr = sequence->GetRotation(frameA);
	auto kfBr = sequence->GetRotation(frameB);
	float cos = DotProduct(kfAr, kfBr);
	if(cos > 1.0f)
		cos = 1.0f;
	theta = Acos(cos);
	invSin = theta == 0.0f ?  0.0f : 1.0f/Sin(theta);
}

void
CAnimBlendNode::GetCurrentTranslation(CVector &trans, float weight)
{
	trans = CVector(0.0f, 0.0f, 0.0f);

	float blend = association->GetBlendAmount(weight);
	if(blend > 0.0f){
		auto kfAdt = sequence->GetDeltaTime(frameA);
		float t = (kfAdt - remainingTime)/kfAdt;
		if(sequence->type & CAnimBlendSequence::KF_TRANS){
			auto kfAt = sequence->GetTranslation(frameA);
			auto kfBt = sequence->GetTranslation(frameB);
			trans = kfBt + t*(kfAt - kfBt);
			trans *= blend;
		}
	}
}

void
CAnimBlendNode::GetEndTranslation(CVector &trans, float weight)
{
	trans = CVector(0.0f, 0.0f, 0.0f);

	float blend = association->GetBlendAmount(weight);
	if(blend > 0.0f){
		if(sequence->type & CAnimBlendSequence::KF_TRANS)
			trans = sequence->GetTranslation(sequence->numFrames-1) * blend;
	}
}
