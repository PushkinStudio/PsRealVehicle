// Copyright 2016 Pushkin Studio. All Rights Reserved.

#include "PrvVehicleMovementComponent.h"

#include "PrvPlugin.h"
#include "PrvVehicleDustEffect.h"

#include "Components/SkinnedMeshComponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/World.h"
#include "Kismet/GameplayStatics.h"
#include "Kismet/KismetMathLibrary.h"
#include "Kismet/KismetSystemLibrary.h"
#include "PhysicsEngine/PhysicsSettings.h"

#include "Runtime/Launch/Resources/Version.h"

DECLARE_CYCLE_STAT(TEXT("Tick Component"), STAT_PrvMovementTickComponent, STATGROUP_MovementPhysics);
DECLARE_CYCLE_STAT(TEXT("Update Steering"), STAT_PrvMovementUpdateSteering, STATGROUP_MovementPhysics);
DECLARE_CYCLE_STAT(TEXT("Update Throttle"), STAT_PrvMovementUpdateThrottle, STATGROUP_MovementPhysics);
DECLARE_CYCLE_STAT(TEXT("Update Gear Box"), STAT_PrvMovementUpdateGearBox, STATGROUP_MovementPhysics);
DECLARE_CYCLE_STAT(TEXT("Update Brake"), STAT_PrvMovementUpdateBrake, STATGROUP_MovementPhysics);
DECLARE_CYCLE_STAT(TEXT("Update Engine"), STAT_PrvMovementUpdateEngine, STATGROUP_MovementPhysics);
DECLARE_CYCLE_STAT(TEXT("Update Suspension"), STAT_PrvMovementUpdateSuspension, STATGROUP_MovementPhysics);
DECLARE_CYCLE_STAT(TEXT("Update Suspension Visuals Only"), STAT_PrvMovementUpdateSuspensionVisualsOnly, STATGROUP_MovementPhysics);
DECLARE_CYCLE_STAT(TEXT("Update Friction"), STAT_PrvMovementUpdateFriction, STATGROUP_MovementPhysics);
DECLARE_CYCLE_STAT(TEXT("Update Wheel Effects"), STAT_PrvMovementUpdateWheelEffects, STATGROUP_MovementPhysics);

static int32 GPrvVehicleShowDustEffect = 0;
static FAutoConsoleVariableRef CVarPrvVehicleShowDustEffect(
	TEXT("PrvVehicle.ShowDustEffect"),
	GPrvVehicleShowDustEffect,
	TEXT("Shows or hides dust effect from vehicle wheels"));

static int32 GPrvVehicleShowDustEffectForOwnerOnly = 1;
static FAutoConsoleVariableRef CVarPrvVehicleShowDustEffectForOwnerOnly(
	TEXT("PrvVehicle.ShowDustEffectForOwnerOnly"),
	GPrvVehicleShowDustEffectForOwnerOnly,
	TEXT("Only owner can see its own wheels dust effect"));

UPrvVehicleMovementComponent::UPrvVehicleMovementComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	bAutoActivate = true;
	bWantsInitializeComponent = true;
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;

	PhysicErrorCorrection.LinearDeltaThresholdSq = 1000000.f;
	PhysicErrorCorrection.LinearInterpAlpha = 0.f;

	bWheeledVehicle = false;
	TransmissionLength = 400.f;
	bOverrideMass = false;
	OverrideVehicleMass = 10000.f;
	LinearDamping = 0.5f;
	AngularDamping = 0.5f;
	COMOffset = FVector::ZeroVector;

	bCustomLinearDamping = false;
	DryFrictionLinearDamping = FVector::ZeroVector;
	FluidFrictionLinearDamping = FVector::ZeroVector;

	bCustomAngularDamping = false;
	DryFrictionAngularDamping = FVector::ZeroVector;
	FluidFrictionAngularDamping = FVector::ZeroVector;

	bLimitEngineTorque = true;
	bIsMovementEnabled = true;
	LastUserSteeringInput = 0;
	bShouldAnimateWheels = true;
	bFakeAutonomousProxy = false;

	SprocketMass = 65.f;
	SprocketRadius = 25.f;
	TrackMass = 600.f;
	SleepLinearVelocity = 5.f;
	SleepAngularVelocity = 5.f;
	SleepDelay = 2.f;
	bDisableGravityForSimulated = true;

	ForceSurfaceType = EPhysicalSurface::SurfaceType_Default;

	bAngularVelocitySteering = true;
	SteeringAngularSpeed = 30.f;
	SteeringUpRatio = 1.f;
	SteeringDownRatio = 1.f;
	AngularSteeringFrictionThreshold = 0.5f;
	AutoBrakeSteeringThreshold = 5000.f;
	bAutoBrakeSteering = false;
	TurnRateModAngularSpeed = 0.f;

	bUseSteeringCurve = false;
	FRichCurve* SteeringCurveData = SteeringCurve.GetRichCurve();
	SteeringCurveData->AddKey(0.f, SteeringAngularSpeed);
	SteeringCurveData->AddKey(2000.f, SteeringAngularSpeed); // 72 Km/h
	SteeringCurveData->AddKey(2500.f, 0.f);
	bMaximizeZeroThrottleSteering = false;

	DefaultWheelBoneOffset = FVector::ZeroVector;
	DefaultLength = 25.f;
	DefaultMaxDrop = 10.f;
	DefaultCollisionRadius = 36.f;
	DefaultCollisionWidth = 20.f;
	DefaultVisualOffset = FVector::ZeroVector;
	VisualCollisionRadius = 36.f;
	DefaultStiffness = 4000000.f;			 // [N/cm]
	DefaultCompressionDamping = 4000000.f;	 // [N/(cm/s)]
	DefaultDecompressionDamping = 4000000.f; // [N/(cm/s)]
	bCustomDampingCorrection = true;
	DampingCorrectionFactor = 1.f;
	bAdaptiveDampingCorrection = true;
	bNotifyRigidBodyCollision = true;
	bTraceComplex = true;

	GearSetup.AddDefaulted(1); // Add at least one gear should exist
	bAutoGear = true;
	bAutoBrake = true;
	bSteeringStabilizer = true;
	SteeringStabilizerMinimumHullVelocity = 10.f;
	SteeringStabilizerBrakeFactor = 0.2f;
	SteeringStabilizerBrakeUpRatio = 1.f;
	SpeedLimitBrakeFactor = 0.1f;
	SpeedLimitBrakeUpRatio = 1.f;

	GearAutoBoxLatency = 0.5f;
	LastAutoGearShiftTime = 0.f;
	LastAutoGearHullSpeed = 0.f;

	ThrottleUpRatio = 0.5f;
	ThrottleDownRatio = 1.f;

	BrakeForce = 30.f;
	AutoBrakeFactor = 1.f;
	SteeringBrakeTransfer = 0.7f;
	SteeringBrakeFactor = 1.f;
	AutoBrakeActivationDelta = 2.f;

	FRichCurve* AutoBrakeCurveData = AutoBrakeUpRatio.GetRichCurve();
	AutoBrakeCurveData->AddKey(0.f, 30.f);
	AutoBrakeCurveData->AddKey(1000.f, 30.f);

	DifferentialRatio = 3.5f;
	TransmissionEfficiency = 0.9f;
	EngineExtraPowerRatio = 3.f;
	EngineRearExtraPowerRatio = 1.f;
	StartExtraPowerRatio = 1.f;
	StartExtraPowerDuration = 0.f;
	StartExtraPowerCooldown = 0.f;
	StartExtraPower = 1.f;
	StartExtraPowerActivationTime = 0.f;
	bStartExtraPowerMovingLast = false;

	bLimitMaxSpeed = false;
	FRichCurve* MaxSpeedCurveData = MaxSpeedCurve.GetRichCurve();
	MaxSpeedCurveData->AddKey(0.f, 2000.f); // 72 Km/h

	TorqueTransferThrottleFactor = 1.f;
	TorqueTransferSteeringFactor = 1.f;

	StaticFrictionCoefficientEllipse = FVector2D(1.f, 1.f);
	KineticFrictionCoefficientEllipse = FVector2D(1.f, 1.f);

	KineticFrictionTorqueCoefficient = 1.f;
	RollingFrictionCoefficient = 0.02f;
	RollingVelocityCoefficientSquared = 0.000015f;
	LinearSpeedPower = 1.f;

	StiffnessFactor = 1.f;
	CompressionDampingFactor = 1.f;
	DecompressionDampingFactor = 1.f;
	DropFactor = 5.f;

	// Init basic torque curve
	FRichCurve* TorqueCurveData = EngineTorqueCurve.GetRichCurve();
	TorqueCurveData->AddKey(0.f, 800.f);
	TorqueCurveData->AddKey(1400.f, 850.f);
	TorqueCurveData->AddKey(2800.f, 800.f);
	TorqueCurveData->AddKey(2810.f, 0.f); // Torque should be zero at max RPM to prevent infinite acceleration

	// Nullify data
	NeutralGear = 0;
	CurrentGear = 0;
	bReverseGear = false;
	LastAutoGearShiftTime = 0.f;
	RightTrackTorque = 0.f;
	LeftTrackTorque = 0.f;

	HullAngularSpeed = 0.f;
	EngineRPM = 0.f;
	EngineTorque = 0.f;
	DriveTorque = 0.f;

	TargetSteeringAngularSpeed = 0.f;
	EffectiveSteeringAngularSpeed = 0.f;
	ActiveFrictionPoints = 0;
	ActiveDrivenFrictionPoints = 0;

	LastSteeringStabilizerBrakeRatio = 0.f;
	LastSpeedLimitBrakeRatio = 0.f;

	UpdatedMesh = nullptr;

	bUseKineticFriction = false;

	CorrectionBeganTime = 0.f;
	CorrectionEndTime = 0.f;
	bCorrectionInProgress = false;

	bUseActiveDrivenFrictionPoints = true;
	bSteeringStabilizerActiveLeft = false;
	bSteeringStabilizerActiveRight = false;

	SuspensionTraceTypeQuery = UEngineTypes::ConvertToTraceType(ECollisionChannel::ECC_Visibility);

	RawSteeringInput = 0.f;
	RawThrottleInput = 0.f;
	LastRawThrottleInput = 0.f;
	RawThrottleInputKeep = 0.f;
	bRawHandbrakeInput = false;
	QuantizeInput = 0;

	bScaleForceToActiveFrictionPoints = false;
	bClampSuspensionForce = false;

	bSimplifiedSuspension = false;
	bSimplifiedSuspensionWithoutThrottle = true;
	bSimplifiedSuspensionByCamera = true;

	bEnableAntiRollover = false;
	AntiRolloverValueThreshold = 1.f;
	FRichCurve* AntiRolloverForceCurveData = AntiRolloverForceCurve.GetRichCurve();
	AntiRolloverForceCurveData->AddKey(0.f, 1000000000.0f);
	AntiRolloverForceCurveData->AddKey(1.f, 20000000000.0f);

	LastAntiRolloverValue = 0.f;
	bUseMeshRotationForEffect = true;
}

//////////////////////////////////////////////////////////////////////////
// Initialization

void UPrvVehicleMovementComponent::InitializeComponent()
{
	Super::InitializeComponent();

	InitMesh();
	InitBodyPhysics();
	CalculateMOI();
	InitSuspension();
	InitGears();

	// Cache RPM limits
	FRichCurve* TorqueCurveData = EngineTorqueCurve.GetRichCurve();
	TorqueCurveData->GetTimeRange(MinEngineRPM, MaxEngineRPM);

	// Be sure that values are higher than zero
	MinEngineRPM = FMath::Max(0.f, MinEngineRPM);
	MaxEngineRPM = FMath::Max(0.f, MaxEngineRPM);
}

void UPrvVehicleMovementComponent::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	PRV_CYCLE_COUNTER(STAT_PrvMovementTickComponent);

	// Notify server about player input
	APawn* MyOwner = UpdatedMesh ? Cast<APawn>(UpdatedMesh->GetOwner()) : nullptr;
	if (MyOwner && MyOwner->IsLocallyControlled())
	{
		const int32 QThrottleInput = FMath::FloorToInt(RawThrottleInput * 127.f) & 0xFF;
		const int32 QSteeringInput = (FMath::FloorToInt(RawSteeringInput * 63.f) & 0x7F) << 8;
		const int32 QHandbrakeInput = bRawHandbrakeInput ? (1 << 15) : 0;
		const uint16 NewQuantizeInput = QHandbrakeInput | QSteeringInput | QThrottleInput;

		if (QuantizeInput != NewQuantizeInput)
		{
			QuantizeInput = NewQuantizeInput;
			ServerUpdateState(QuantizeInput);
		}
	}

	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// Check that mesh exists
	if (!UpdatedMesh)
	{
		return;
	}

	// Reset sleeping state each time we have any input
	if (HasInput())
	{
		ResetSleep();
	}

	// Check we're not sleeping (don't update physics state while sleeping)
	if (!IsSleeping(DeltaTime))
	{
		// Perform full simulation only on server and for local owner
		if (ShouldAddForce())
		{
			// Suspension
			UpdateSuspension(DeltaTime);
			UpdateFriction(DeltaTime);

			// Engine
			UpdateSteering(DeltaTime);
			UpdateThrottle(DeltaTime);

			// Control
			UpdateGearBox();
			UpdateBrake(DeltaTime);

			// Movement
			UpdateTracksVelocity(DeltaTime);
			UpdateHullVelocity(DeltaTime);
			UpdateEngineStartExtraPower(DeltaTime);
			UpdateEngine();
			UpdateDriveForce();

			// Additional damping
			UpdateLinearVelocity(DeltaTime);
			UpdateAngularVelocity(DeltaTime);

			if (bEnableAntiRollover)
			{
				UpdateAntiRollover(DeltaTime);
			}

			UpdateReplicatedCosmeticData();
		}
		else
		{
			// Check that wheels should be animated anyway
			UpdateSuspensionVisualsOnly(DeltaTime);

			// Disable gravity for ROLE_SimulatedProxy or fake autonomous ones
			if (bDisableGravityForSimulated && UpdatedMesh->IsGravityEnabled())
			{
				UpdatedMesh->SetEnableGravity(false);
			}

			// Check if we are in the process of body's state correction
			if (bCorrectionInProgress && GetWorld()->GetTimeSeconds() >= CorrectionEndTime)
			{
				// Time has come
				// Set the body into it's meant position

				bCorrectionInProgress = false;

				FVector DeltaPos(FVector::ZeroVector);
				ErrorCorrectionData.LinearDeltaThresholdSq /= 2.f;
				ErrorCorrectionData.AngularDeltaThreshold /= 2.f;
				ErrorCorrectionData.LinearRecipFixTime *= 2.f;
				ErrorCorrectionData.AngularRecipFixTime *= 2.f;

				UE_LOG(LogPrvVehicle, Verbose, TEXT("Force correct body position, LinearRecipFixTime=%.2f"), ErrorCorrectionData.LinearRecipFixTime);

				ApplyRigidBodyState(CorrectionEndState, ErrorCorrectionData, DeltaPos);
			}
		}
	}

	// @todo Network wheels animation
	AnimateWheels(DeltaTime);

	// Update dust VFX
	if (!IsRunningDedicatedServer())
	{
		UpdateWheelEffects(DeltaTime);
	}

	// Show debug
	if (bShowDebug)
	{
		DrawDebugLines();
	}
}

//////////////////////////////////////////////////////////////////////////
// Physics Initialization

void UPrvVehicleMovementComponent::InitMesh()
{
	if (!UpdatedMesh)
	{
		UpdatedMesh = Cast<USkinnedMeshComponent>(UpdatedComponent);
	}
}

void UPrvVehicleMovementComponent::InitBodyPhysics()
{
	if (!UpdatedMesh)
	{
		UE_LOG(LogPrvVehicle, Error, TEXT("InitBodyPhysics failed: No UpdatedMesh component found"));
		return;
	}

	if (bOverrideMass)
	{
		UpdatedMesh->SetMassOverrideInKg(NAME_None, OverrideVehicleMass);
	}

	if (!bCustomLinearDamping)
	{
		UpdatedMesh->SetLinearDamping(LinearDamping);
	}
	else
	{
		// Force zero physX damping
		UpdatedMesh->SetLinearDamping(0.f);
	}

	if (!bCustomAngularDamping)
	{
		UpdatedMesh->SetAngularDamping(AngularDamping);
	}
	else
	{
		// Force zero damping instead
		UpdatedMesh->SetAngularDamping(0.f);
	}

	UpdatedMesh->SetCenterOfMass(COMOffset);
}

void UPrvVehicleMovementComponent::InitSuspension()
{
	if (!UpdatedMesh)
	{
		UE_LOG(LogPrvVehicle, Error, TEXT("InitSuspension failed: No UpdatedMesh component found"));
		return;
	}

	for (auto& SuspInfo : SuspensionSetup)
	{
		if (!SuspInfo.bCustomWheelConfig)
		{
			SuspInfo.WheelBoneOffset = DefaultWheelBoneOffset;
			SuspInfo.Length = DefaultLength;
			SuspInfo.MaxDrop = DefaultMaxDrop;
			SuspInfo.CollisionRadius = DefaultCollisionRadius;
			SuspInfo.CollisionWidth = DefaultCollisionWidth;
			SuspInfo.VisualOffset = DefaultVisualOffset;
			SuspInfo.Stiffness = DefaultStiffness;
			SuspInfo.CompressionDamping = DefaultCompressionDamping;
			SuspInfo.DecompressionDamping = DefaultDecompressionDamping;

			if (SuspInfo.bRightTrack)
			{
				SuspInfo.WheelBoneOffset.Y *= -1.f;
				SuspInfo.VisualOffset.Y *= -1.f;
			}
		}

		if (UpdatedMesh)
		{
			if (SuspInfo.bInheritWheelBoneTransform)
			{
				FTransform WheelTransform = UpdatedMesh->GetSocketTransform(SuspInfo.BoneName, RTS_Actor);
				SuspInfo.Location = WheelTransform.GetLocation() + SuspInfo.WheelBoneOffset + FVector::UpVector * SuspInfo.Length;
				SuspInfo.Rotation = WheelTransform.GetRotation().Rotator();

				UE_LOG(LogPrvVehicle, Log, TEXT("Init suspension (%s): %s"), *SuspInfo.BoneName.ToString(), *SuspInfo.Location.ToString());
			}
			else
			{
				FTransform WheelTransform = UpdatedMesh->GetSocketTransform(SuspInfo.BoneName, RTS_Actor);
				SuspInfo.WheelBoneOffset = (SuspInfo.Location - FVector::UpVector * SuspInfo.Length) - WheelTransform.GetLocation();
			}
		}

		FSuspensionState SuspState;
		SuspState.SuspensionInfo = SuspInfo;
		SuspState.PreviousLength = SuspInfo.Length;
		SuspState.VisualLength = DefaultLength;

		if (SuspInfo.bSpawnDust)
		{
			SuspState.DustPSC = SpawnNewWheelEffect();
		}

		SuspensionData.Add(SuspState);
	}
}

void UPrvVehicleMovementComponent::InitGears()
{
	for (int32 i = 0; i < GearSetup.Num(); ++i)
	{
		if (FMath::IsNearlyZero(GearSetup[i].Ratio))
		{
			NeutralGear = i;
			break;
		}
	}

	// Start with neutral gear
	CurrentGear = NeutralGear;

	UE_LOG(LogPrvVehicle, Warning, TEXT("Neutral gear: %d"), NeutralGear);
}

void UPrvVehicleMovementComponent::CalculateMOI()
{
	if (!UpdatedMesh)
	{
		UE_LOG(LogPrvVehicle, Error, TEXT("CalculateMOI failed: No UpdatedMesh component found"));
		return;
	}

	const float SprocketSquareRadius = (SprocketRadius * SprocketRadius);
	const float SprocketMOI = (SprocketMass / 2) * SprocketSquareRadius;
	const float TrackMOI = TrackMass * SprocketSquareRadius;

	FinalMOI = SprocketMOI + TrackMOI;

	UE_LOG(LogPrvVehicle, Warning, TEXT("Final MOI: %f"), FinalMOI);
	UE_LOG(LogPrvVehicle, Warning, TEXT("Vehicle mass: %f"), UpdatedMesh->GetMass());
}

//////////////////////////////////////////////////////////////////////////
// Physics simulation

bool UPrvVehicleMovementComponent::IsSleeping(float DeltaTime)
{
	// Force sleeping if mesh isn't simulate physics
	if (UpdatedMesh && !UpdatedMesh->IsSimulatingPhysics())
	{
		if (!bIsSleeping)
		{
			bIsSleeping = true;

			// Force update on server
			OnRep_IsSleeping();
		}

		return true;
	}

	if (bForceNeverSleep)
	{
		ResetSleep();
		return false;
	}

	if (!bIsSleeping && (SleepTimer < SleepDelay))
	{
		SleepTimer += DeltaTime;
		return false;
	}

	if (UpdatedMesh &&
		UpdatedMesh->GetPhysicsLinearVelocity().SizeSquared() < SleepLinearVelocity &&
		UpdatedMesh->GetPhysicsAngularVelocityInDegrees().SizeSquared() < SleepAngularVelocity)
	{
		if (!bIsSleeping)
		{
			bIsSleeping = true;

			// Force update on server
			OnRep_IsSleeping();
		}
	}
	else
	{
		ResetSleep();
	}

	return bIsSleeping;
}

void UPrvVehicleMovementComponent::ResetSleep()
{
	bIsSleeping = false;
	SleepTimer = 0.f;
}

void UPrvVehicleMovementComponent::OnRep_IsSleeping()
{
	if (bIsSleeping)
	{
		SleepTimer = 0.f;
		UpdatedMesh->PutAllRigidBodiesToSleep();
	}
}

void UPrvVehicleMovementComponent::UpdateSteering(float DeltaTime)
{
	PRV_CYCLE_COUNTER(STAT_PrvMovementUpdateSteering);

	bool bFullSteeringFriction = true;
	float FrictionRatio = 1.f;

	if (bAngularVelocitySteering)
	{
		if (bUseActiveDrivenFrictionPoints && SuspensionData.Num() > 0)
		{
			FrictionRatio = static_cast<float>(ActiveDrivenFrictionPoints) / SuspensionData.Num();

			if (FrictionRatio >= AngularSteeringFrictionThreshold)
			{
				FrictionRatio = 1.f;
			}
			else
			{
				bFullSteeringFriction = false;
			}
		}

		const bool bSteeringUp = FMath::RoundToInt(FMath::Sign(SteeringInput)) == FMath::RoundToInt(FMath::Sign(RawSteeringInput));

		// Don't add SteeringInput when insufficient number of wheel are touching the ground
		if (bFullSteeringFriction || bSteeringUp == false)
		{
			if (FMath::IsNearlyZero(RawSteeringInput) == false)
			{
				// Steering ratio depends on current steering direction
				const float SteeringChangeRatio = bSteeringUp ? SteeringUpRatio : SteeringDownRatio;

				// -- [Car] --
				if (bWheeledVehicle)
				{
					SteeringInput = SteeringInput + FMath::Sign(RawSteeringInput) * SteeringChangeRatio * DeltaTime;
				}
				// -- [Tank] --
				else
				{
					const float InputSign = (RawThrottleInput < 0.f) ? -1.f : 1.f;

					SteeringInput = SteeringInput + InputSign * FMath::Sign(RawSteeringInput) * SteeringChangeRatio * DeltaTime;
				}

				// Clamp steering to joystick values
				SteeringInput = FMath::Clamp(
					SteeringInput,
					(-1.f) * FMath::Abs(RawSteeringInput),
					FMath::Abs(RawSteeringInput));
			}
			else
			{
				SteeringInput = FMath::Sign(SteeringInput) * FMath::Max(0.f, (FMath::Abs(SteeringInput) - (SteeringDownRatio * DeltaTime)));
			}
		}

		// No direct input to tracks
		LeftTrack.Input = 0.f;
		RightTrack.Input = 0.f;
	}
	else
	{
		SteeringInput = RawSteeringInput;
		LeftTrack.Input = SteeringInput;
		RightTrack.Input = -SteeringInput;
	}

	const float CurrentSpeed = UpdatedMesh->GetComponentVelocity().Size();

	if (bUseSteeringCurve)
	{
		FRichCurve* SteeringCurveData = SteeringCurve.GetRichCurve();
		const float SteeringCurveZeroPoint = FMath::Min(SteeringCurveData->Eval(0.f) + TurnRateModAngularSpeed, SteeringAngularSpeed);
		const float SteeringCurvePoint = FMath::Min(SteeringCurveData->Eval(GetForwardSpeed()) + TurnRateModAngularSpeed, SteeringAngularSpeed);

		if (bMaximizeZeroThrottleSteering && FMath::IsNearlyZero(RawThrottleInput))
		{
			const float BiggerCurveValue = FMath::Max(SteeringCurveZeroPoint, SteeringCurvePoint);
			TargetSteeringAngularSpeed = SteeringInput * BiggerCurveValue;
			EffectiveSteeringAngularSpeed = TargetSteeringAngularSpeed;
		}
		else
		{
			// Check steering limitation (issue #51 magic)
			if (bLimitMaxSpeed)
			{
				if (FMath::IsNearlyZero(SteeringInput) == false)
				{
					TargetSteeringAngularSpeed = SteeringInput * SteeringCurveZeroPoint;
					EffectiveSteeringAngularSpeed = SteeringCurvePoint * SteeringInput;
				}
				else
				{
					EffectiveSteeringAngularSpeed = 0.f;
					TargetSteeringAngularSpeed = 0.f;
				}
			}
			else
			{
				EffectiveSteeringAngularSpeed = SteeringInput * SteeringCurvePoint;
				TargetSteeringAngularSpeed = EffectiveSteeringAngularSpeed;
			}
		}
	}
	else
	{
		EffectiveSteeringAngularSpeed = SteeringInput * SteeringAngularSpeed;
		TargetSteeringAngularSpeed = EffectiveSteeringAngularSpeed;
	}

	// If speed is above threshold and we are in "full steering" position, apply steering by autobrake instead of angular velocity
	bAutoBrakeSteering = (CurrentSpeed >= AutoBrakeSteeringThreshold && FMath::IsNearlyEqual(FMath::Abs(RawSteeringInput), 1.f) && FMath::IsNearlyZero(RawThrottleInput));

	if (bAngularVelocitySteering)
	{
		FVector LocalAngularVelocity = UpdatedMesh->GetComponentTransform().InverseTransformVectorNoScale(UpdatedMesh->GetPhysicsAngularVelocityInDegrees());

		float TargetSteeringVelocity = EffectiveSteeringAngularSpeed;

		if (bUseActiveDrivenFrictionPoints)
		{
			TargetSteeringVelocity *= FrictionRatio;
		}

		// -- [Car] --
		if (bWheeledVehicle)
		{
			// Simple model of angular speed for car
			const float TargetSteeringVelocitySin = FMath::Sin(FMath::DegreesToRadians(TargetSteeringVelocity));
			if (FMath::IsNearlyZero(TargetSteeringVelocitySin) == false)
			{
				const float TurnRadius = TransmissionLength / TargetSteeringVelocitySin;
				if (FMath::IsNearlyZero(TurnRadius) == false)
				{
					const FVector NormalizedVelocity = UpdatedMesh->GetComponentVelocity().GetSafeNormal();
					const float SpeedXProjection = GetForwardSpeed() * FMath::Abs(FVector::DotProduct(UpdatedMesh->GetForwardVector(), NormalizedVelocity));
					TargetSteeringVelocity = FMath::RadiansToDegrees(SpeedXProjection / TurnRadius);
				}
			}
		}

		if (FMath::IsNearlyZero(RawSteeringInput) == false)
		{
			const bool bShouldSet = bAutoBrakeSteering ? (FMath::Abs(LocalAngularVelocity.Z) < FMath::Abs(TargetSteeringVelocity)) : true;

			if (ShouldAddForce() && bShouldSet && bFullSteeringFriction)
			{
				LocalAngularVelocity.Z = TargetSteeringVelocity;
				EffectiveSteeringVelocity = UpdatedMesh->GetComponentTransform().TransformVectorNoScale(LocalAngularVelocity);
				UpdatedMesh->SetPhysicsAngularVelocityInDegrees(EffectiveSteeringVelocity);
			}
		}
		else
		{
			EffectiveSteeringVelocity = FVector::ZeroVector;
		}
	}

	// -- [Car] --
	if (bWheeledVehicle && bFullSteeringFriction)
	{
		// Update driving wheels for wheeled vehicles
		for (auto& SuspState : SuspensionData)
		{
			if (SuspState.SuspensionInfo.bSteeringWheel)
			{
				SuspState.SuspensionInfo.Rotation.Yaw = EffectiveSteeringAngularSpeed;
			}
		}
	}
}

void UPrvVehicleMovementComponent::UpdateThrottle(float DeltaTime)
{
	PRV_CYCLE_COUNTER(STAT_PrvMovementUpdateThrottle);

	// -- [Car] --
	if (bWheeledVehicle)
	{
		LeftTrack.TorqueTransfer = FMath::Abs(RawThrottleInput) * TorqueTransferThrottleFactor;
		RightTrack.TorqueTransfer = FMath::Abs(RawThrottleInput) * TorqueTransferThrottleFactor;
	}
	// -- [Tank] --
	else
	{
		// Calc torque transfer based on input
		if (FMath::Abs(RawThrottleInput) > SMALL_NUMBER)
		{
			LeftTrack.TorqueTransfer = FMath::Abs(RawThrottleInput) * TorqueTransferThrottleFactor + FMath::Max(0.f, LeftTrack.Input) * TorqueTransferSteeringFactor;
			RightTrack.TorqueTransfer = FMath::Abs(RawThrottleInput) * TorqueTransferThrottleFactor + FMath::Max(0.f, RightTrack.Input) * TorqueTransferSteeringFactor;
		}
		else
		{
			LeftTrack.TorqueTransfer = FMath::Abs(RawThrottleInput) * TorqueTransferThrottleFactor + LeftTrack.Input * TorqueTransferSteeringFactor;
			RightTrack.TorqueTransfer = FMath::Abs(RawThrottleInput) * TorqueTransferThrottleFactor + RightTrack.Input * TorqueTransferSteeringFactor;
		}
	}

	// Throttle shouldn't be instant
	if (FMath::Abs(LeftTrack.TorqueTransfer) > SMALL_NUMBER || FMath::Abs(RightTrack.TorqueTransfer) > SMALL_NUMBER)
	{
		ThrottleInput += (ThrottleUpRatio * DeltaTime);
	}
	else
	{
		ThrottleInput -= (ThrottleDownRatio * DeltaTime);
	}

#if !PLATFORM_SWITCH
	// Limit throttle to [0; 1]
	ThrottleInput = FMath::Clamp(ThrottleInput, 0.f, 1.0f);
#else
	ThrottleInput = FMath::Clamp(ThrottleInput, 0.f, RawThrottleInput);
#endif //!PLATFORM_SWITCH

	// Debug
	if (bShowDebug)
	{
		// Torque transfer balance
		DrawDebugString(GetWorld(), UpdatedMesh->GetComponentTransform().TransformPosition(FVector(0.f, -100.f, 0.f)), FString::SanitizeFloat(LeftTrack.TorqueTransfer), nullptr, FColor::White, 0.f);
		DrawDebugString(GetWorld(), UpdatedMesh->GetComponentTransform().TransformPosition(FVector(0.f, 100.f, 0.f)), FString::SanitizeFloat(RightTrack.TorqueTransfer), nullptr, FColor::White, 0.f);
	}
}

void UPrvVehicleMovementComponent::UpdateGearBox()
{
	PRV_CYCLE_COUNTER(STAT_PrvMovementUpdateGearBox);

	if (!bAutoGear)
	{
		return;
	}

	const bool bHasThrottleInput = FMath::IsNearlyZero(RawThrottleInput) == false;
	const bool bHasSteeringInput = FMath::IsNearlyZero(SteeringInput) == false;

	// With auto-gear we shouldn't have neutral
	if (CurrentGear == NeutralGear && (bHasThrottleInput || bHasSteeringInput))
	{
		ShiftGear(RawThrottleInput >= 0.f);
	}

	const bool bIsMovingForward = (FVector::DotProduct(UpdatedMesh->GetForwardVector(), UpdatedMesh->GetComponentVelocity()) >= 0.f);
	const bool bHasAppropriateGear = ((RawThrottleInput <= 0.f) == bReverseGear);

	// Force switch gears on input direction change
	if (bHasThrottleInput && !bHasAppropriateGear)
	{
		ShiftGear(!bIsMovingForward);
	}
	// Check that we can shift gear by time
	else if ((GetWorld()->GetTimeSeconds() - LastAutoGearShiftTime) > GearAutoBoxLatency)
	{
		const float CurrentRPMRatio = (EngineRPM - MinEngineRPM) / (MaxEngineRPM - MinEngineRPM);

		if (CurrentRPMRatio >= GetCurrentGearInfo().UpRatio)
		{
			// Shift up
			ShiftGear(!bReverseGear);
		}
		else if (CurrentRPMRatio <= GetCurrentGearInfo().DownRatio)
		{
			// Shift down
			ShiftGear(bReverseGear);
		}
	}

	LastAutoGearHullSpeed = HullAngularSpeed;
}

void UPrvVehicleMovementComponent::ShiftGear(bool bShiftUp)
{
	const int32 PrevGear = CurrentGear;

	if (bShiftUp)
	{
		CurrentGear += 1;
	}
	else
	{
		CurrentGear -= 1;
	}

	CurrentGear = FMath::Clamp(CurrentGear, 0, GearSetup.Num() - 1);

	// Force gears limits on user input
	if (FMath::IsNearlyZero(RawThrottleInput) == false)
	{
		bReverseGear = (RawThrottleInput < 0.f);

		if (bReverseGear)
		{
			CurrentGear = FMath::Max(0, FMath::Min(CurrentGear, NeutralGear - 1));
		}
		else
		{
			CurrentGear = FMath::Max(CurrentGear, NeutralGear);
		}
	}
	else
	{
		// Don't switch gear when we want to be neutral
		if (PrevGear >= NeutralGear)
		{
			CurrentGear = FMath::Max(CurrentGear, NeutralGear);
		}
		else
		{
			CurrentGear = FMath::Min(CurrentGear, NeutralGear);
		}

		bReverseGear = (CurrentGear < NeutralGear);
	}

	if (bDebugAutoGearBox)
	{
		if (bShiftUp)
		{
			UE_LOG(LogPrvVehicle, Warning, TEXT("Switch gear up: was %d, now %d"), PrevGear, CurrentGear);
		}
		else
		{
			UE_LOG(LogPrvVehicle, Warning, TEXT("Switch gear down: was %d, now %d"), PrevGear, CurrentGear);
		}
	}

	LastAutoGearShiftTime = GetWorld()->GetTimeSeconds();
}

void UPrvVehicleMovementComponent::UpdateBrake(float DeltaTime)
{
	PRV_CYCLE_COUNTER(STAT_PrvMovementUpdateBrake);

	float BrakeInputIncremented = 0.f;
	const bool bIsMovingForward = FVector::DotProduct(UpdatedMesh->GetForwardVector(), UpdatedMesh->GetComponentVelocity()) >= 0.f;

	if (bAutoBrake)
	{
		const float AutoBrakeCurveValue = AutoBrakeUpRatio.GetRichCurve()->Eval(GetForwardSpeed());
		BrakeInputIncremented = FMath::Clamp(BrakeInput + AutoBrakeCurveValue * DeltaTime, 0.f, AutoBrakeFactor);
		const bool bHasThrottleInput = (FMath::IsNearlyZero(RawThrottleInput) == false);

		if (bHasThrottleInput == false && (bAngularVelocitySteering || FMath::IsNearlyZero(SteeringInput)))
		{
			BrakeInput = BrakeInputIncremented;
		}
		else
		{
			const bool bMovingThrottleInputDirection = (bIsMovingForward == (RawThrottleInput > 0.f));
			const bool bNonZeroAngularVelocity =
				(FMath::IsNearlyZero(FMath::Sign(LeftTrack.AngularSpeed)) == false) &&
				(FMath::IsNearlyZero(FMath::Sign(RightTrack.AngularSpeed)) == false);
			const bool bWrongAngularVelocityDirection =
				(FMath::RoundToInt(FMath::Sign(LeftTrack.AngularSpeed)) != FMath::RoundToInt(FMath::Sign(RawThrottleInput))) &&
				(FMath::RoundToInt(FMath::Sign(RightTrack.AngularSpeed)) != FMath::RoundToInt(FMath::Sign(RawThrottleInput)));

			// Brake when direction is changing
			if (bHasThrottleInput && !bMovingThrottleInputDirection && bNonZeroAngularVelocity && bWrongAngularVelocityDirection)
			{
				BrakeInput = BrakeInputIncremented;
			}
			else
			{
				BrakeInput = BrakeInputIncremented * bRawHandbrakeInput;
			}
		}
	}

	// Handbrake first
	LeftTrack.BrakeRatio = BrakeInput;
	RightTrack.BrakeRatio = BrakeInput;

	// -- [Tank] --
	if (bWheeledVehicle == false && bAngularVelocitySteering == false && FMath::IsNearlyZero(RawThrottleInput) == false)
	{
		// Manual brake for rotation
		if ((LeftTrack.Input < 0.f) && (FMath::Abs(LeftTrack.AngularSpeed) >= FMath::Abs(RightTrack.AngularSpeed * SteeringBrakeTransfer)))
		{
			LeftTrack.BrakeRatio = (-1.f) * LeftTrack.Input * SteeringBrakeFactor;
		}
		else if ((RightTrack.Input < 0.f) && (FMath::Abs(RightTrack.AngularSpeed) >= FMath::Abs(LeftTrack.AngularSpeed * SteeringBrakeTransfer)))
		{
			RightTrack.BrakeRatio = (-1.f) * RightTrack.Input * SteeringBrakeFactor;
		}
	}

	const bool bSteeringStabilizerActive = (bSteeringStabilizerActiveLeft || bSteeringStabilizerActiveRight);

	// Stabilize steering
	bSteeringStabilizerActiveLeft = false;
	bSteeringStabilizerActiveRight = false;

	if (bSteeringStabilizer && FMath::IsNearlyZero(SteeringInput) &&
		(HullAngularSpeed > SteeringStabilizerMinimumHullVelocity))
	{
		// Smooth brake ratio up
		LastSteeringStabilizerBrakeRatio += (SteeringStabilizerBrakeUpRatio * DeltaTime);
		LastSteeringStabilizerBrakeRatio = FMath::Clamp(LastSteeringStabilizerBrakeRatio, 0.f, SteeringStabilizerBrakeFactor);

		// Apply brake or reset it
		if (FMath::Abs(LeftTrack.AngularSpeed) - FMath::Abs(RightTrack.AngularSpeed) > AutoBrakeActivationDelta)
		{
			LeftTrack.BrakeRatio = LastSteeringStabilizerBrakeRatio;
			RightTrack.BrakeRatio = 0.f;
			bSteeringStabilizerActiveLeft = true;
		}
		else if (FMath::Abs(RightTrack.AngularSpeed) - FMath::Abs(LeftTrack.AngularSpeed) > AutoBrakeActivationDelta)
		{
			RightTrack.BrakeRatio = LastSteeringStabilizerBrakeRatio;
			LeftTrack.BrakeRatio = 0.f;
			bSteeringStabilizerActiveRight = true;
		}
		else
		{
			LastSteeringStabilizerBrakeRatio = 0.f;
		}
	}
	else
	{
		LastSteeringStabilizerBrakeRatio = 0.f;
	}

	const bool bSteeringStabilizerActiveAfter = (bSteeringStabilizerActiveLeft || bSteeringStabilizerActiveRight);

	if (bSteeringStabilizerActive && !bSteeringStabilizerActiveAfter)
	{
		SetThrottleInput(RawThrottleInputKeep);
	}

	// Brake on speed limitation when steering
	if (FMath::IsNearlyZero(LeftTrack.BrakeRatio) && FMath::IsNearlyZero(RightTrack.BrakeRatio) &&
		bLimitMaxSpeed &&
		FMath::IsNearlyZero(EffectiveSteeringAngularSpeed) == false)
	{
		const float CurrentSpeed = UpdatedMesh->GetComponentVelocity().Size();

		FRichCurve* MaxSpeedCurveData = MaxSpeedCurve.GetRichCurve();
		const float MaxSpeedLimit = MaxSpeedCurveData->Eval(FMath::Abs(TargetSteeringAngularSpeed) - TurnRateModAngularSpeed);

		if (CurrentSpeed >= MaxSpeedLimit)
		{
			// Smooth brake ratio up
			LastSpeedLimitBrakeRatio += (SpeedLimitBrakeUpRatio * DeltaTime);
			LastSpeedLimitBrakeRatio = FMath::Clamp(LastSpeedLimitBrakeRatio, 0.f, SpeedLimitBrakeFactor);

			LeftTrack.BrakeRatio = LastSpeedLimitBrakeRatio;
			RightTrack.BrakeRatio = LastSpeedLimitBrakeRatio;
		}
		else
		{
			// Reset brake if no speed limitation occured
			LastSpeedLimitBrakeRatio = 0.f;
		}
	}
	else
	{
		LastSpeedLimitBrakeRatio = 0.f;
	}

	if (bAutoBrakeSteering && bAutoBrake)
	{
		BrakeInput = BrakeInputIncremented;

		if ((SteeringInput > 0.f && bIsMovingForward) ||
			(SteeringInput < 0.f && bIsMovingForward == false))
		{
			LeftTrack.BrakeRatio = 0.f;
			RightTrack.BrakeRatio = BrakeInput;
		}
		else
		{
			LeftTrack.BrakeRatio = BrakeInput;
			RightTrack.BrakeRatio = 0.f;
		}
	}
}

void UPrvVehicleMovementComponent::UpdateTracksVelocity(float DeltaTime)
{
	// Calc total torque
	RightTrackTorque = RightTrack.DriveTorque + RightTrack.KineticFrictionTorque + RightTrack.RollingFrictionTorque;
	LeftTrackTorque = LeftTrack.DriveTorque + LeftTrack.KineticFrictionTorque + LeftTrack.RollingFrictionTorque;

	// Update right track velocity
	const float RightAngularSpeed = RightTrack.AngularSpeed + (bUseKineticFriction ? (RightTrackTorque / FinalMOI * DeltaTime) : 0.f);
	RightTrackEffectiveAngularSpeed = RightAngularSpeed;
	RightTrack.AngularSpeed = ApplyBrake(DeltaTime, RightAngularSpeed, RightTrack.BrakeRatio);
	RightTrack.LinearSpeed = RightTrack.AngularSpeed * SprocketRadius;

	// Update left track velocity
	const float LeftAngularSpeed = LeftTrack.AngularSpeed + (bUseKineticFriction ? (LeftTrackTorque / FinalMOI * DeltaTime) : 0.f);
	LeftTrackEffectiveAngularSpeed = LeftAngularSpeed;
	LeftTrack.AngularSpeed = ApplyBrake(DeltaTime, LeftAngularSpeed, LeftTrack.BrakeRatio);
	LeftTrack.LinearSpeed = LeftTrack.AngularSpeed * SprocketRadius;

	// Debug
	if (bShowDebug)
	{
		// Tracks torque
		DrawDebugString(GetWorld(), UpdatedMesh->GetComponentTransform().TransformPosition(FVector(0.f, -300.f, 0.f)), FString::SanitizeFloat(LeftTrackTorque), nullptr, FColor::White, 0.f);
		DrawDebugString(GetWorld(), UpdatedMesh->GetComponentTransform().TransformPosition(FVector(0.f, 300.f, 0.f)), FString::SanitizeFloat(RightTrackTorque), nullptr, FColor::White, 0.f);

		// Tracks torque
		DrawDebugString(GetWorld(), UpdatedMesh->GetComponentTransform().TransformPosition(FVector(0.f, -500.f, 0.f)), FString::SanitizeFloat(LeftTrack.AngularSpeed), nullptr, FColor::White, 0.f);
		DrawDebugString(GetWorld(), UpdatedMesh->GetComponentTransform().TransformPosition(FVector(0.f, 500.f, 0.f)), FString::SanitizeFloat(RightTrack.AngularSpeed), nullptr, FColor::White, 0.f);
	}
}

float UPrvVehicleMovementComponent::ApplyBrake(float DeltaTime, float AngularVelocity, float BrakeRatio)
{
	const float BrakeVelocity = BrakeRatio * BrakeForce * DeltaTime;

	if (FMath::Abs(AngularVelocity) > FMath::Abs(BrakeVelocity))
	{
		return (AngularVelocity - (BrakeVelocity * FMath::Sign(AngularVelocity)));
	}

	return 0.f;
}

void UPrvVehicleMovementComponent::UpdateHullVelocity(float DeltaTime)
{
	HullAngularSpeed = (FMath::Abs(LeftTrack.AngularSpeed) + FMath::Abs(RightTrack.AngularSpeed)) / 2.f;
}

void UPrvVehicleMovementComponent::UpdateEngineStartExtraPower(float DeltaTime)
{
	const float CurrentTime = GetWorld()->GetTimeSeconds();
	if (CurrentTime - StartExtraPowerActivationTime >= StartExtraPowerDuration)
	{
		StartExtraPower = 1.f;
	}

	const float CurrentSpeed = UpdatedMesh->GetComponentVelocity().Size();
	const bool bMoving = !FMath::IsNearlyZero(CurrentSpeed, 1.f) && !FMath::IsNearlyZero(FMath::Abs(RawThrottleInput));
	const bool bStarted = !bStartExtraPowerMovingLast && bMoving && !FMath::IsNearlyZero(FMath::Abs(RawThrottleInput));
	const bool bCanStartAfterTimeout = bStarted && (FMath::IsNearlyZero(StartExtraPowerActivationTime) || (CurrentTime - StartExtraPowerActivationTime >= StartExtraPowerCooldown));

	const float SpeedSign = FMath::Sign(FVector::DotProduct(UpdatedMesh->GetForwardVector(), UpdatedMesh->GetComponentVelocity()));
	const bool bWantToMoveOppositeDirection = bMoving && !bStarted && (FMath::Sign(RawThrottleInput) * SpeedSign < 0.f);

	if (bWantToMoveOppositeDirection || bCanStartAfterTimeout)
	{
		StartExtraPowerActivationTime = CurrentTime;
		StartExtraPower = StartExtraPowerRatio;
	}

	bStartExtraPowerMovingLast = bMoving;
}

void UPrvVehicleMovementComponent::UpdateEngine()
{
	PRV_CYCLE_COUNTER(STAT_PrvMovementUpdateEngine);

	const FGearInfo CurrentGearInfo = GetCurrentGearInfo();

	// Update engine rotation speed (RPM)
	EngineRPM = PrvOmegaToRPM((CurrentGearInfo.Ratio * DifferentialRatio) * HullAngularSpeed);
	EngineRPM = FMath::Clamp(EngineRPM, MinEngineRPM, MaxEngineRPM);

	// Calculate engine torque based on current RPM
	FRichCurve* TorqueCurveData = EngineTorqueCurve.GetRichCurve();
	const float MaxEngineTorque = TorqueCurveData->Eval(EngineRPM) * 100.f; // Meters to Cm

	// Check engine torque limitations
	const float CurrentSpeed = UpdatedMesh->GetComponentVelocity().Size();
	const bool bLimitTorqueByRPM = bLimitEngineTorque && FMath::Abs(EngineRPM - MaxEngineRPM) < SMALL_NUMBER;

	// Check steering limitation
	bool bLimitTorqueBySpeed = false;
	if (bLimitMaxSpeed)
	{
		FRichCurve* MaxSpeedCurveData = MaxSpeedCurve.GetRichCurve();
		const float MaxSpeedLimit = MaxSpeedCurveData->Eval(FMath::Abs(TargetSteeringAngularSpeed) - TurnRateModAngularSpeed);

		bLimitTorqueBySpeed = (CurrentSpeed >= MaxSpeedLimit);
	}

	// Check we've reached the limit
	if (bLimitTorqueBySpeed || bLimitTorqueByRPM)
	{
		EngineTorque = 0.f;
	}
	else
	{
		EngineTorque = MaxEngineTorque * ThrottleInput;
	}

	// Gear box torque
	DriveTorque = EngineTorque * CurrentGearInfo.Ratio * DifferentialRatio * TransmissionEfficiency;
	DriveTorque *= (bReverseGear) ? -1.f : 1.f;
	DriveTorque *= EngineExtraPowerRatio;

	if (RawThrottleInput < 0.f)
	{
		DriveTorque *= EngineRearExtraPowerRatio;
	}

	DriveTorque *= StartExtraPower;

	// Debug
	if (bShowDebug)
	{
		DrawDebugString(GetWorld(), UpdatedMesh->GetComponentTransform().TransformPosition(FVector(0.f, 0.f, 200.f)), FString::SanitizeFloat(EngineRPM), nullptr, FColor::Red, 0.f);
		DrawDebugString(GetWorld(), UpdatedMesh->GetComponentTransform().TransformPosition(FVector(0.f, 0.f, 250.f)), FString::SanitizeFloat(MaxEngineTorque), nullptr, FColor::White, 0.f);
		DrawDebugString(GetWorld(), UpdatedMesh->GetComponentTransform().TransformPosition(FVector(0.f, 0.f, 300.f)), FString::SanitizeFloat(DriveTorque), nullptr, FColor::Red, 0.f);
	}
}

void UPrvVehicleMovementComponent::UpdateDriveForce()
{
	// Drive force (right)
	if (bSteeringStabilizerActiveRight == false)
	{
		RightTrack.DriveTorque = RightTrack.TorqueTransfer * DriveTorque;
		RightTrack.DriveForce = UpdatedMesh->GetForwardVector() * (RightTrackTorque / SprocketRadius);
	}
	else
	{
		RightTrack.DriveTorque = 0.f;
		RightTrack.DriveForce = FVector::ZeroVector;
	}

	// Drive force (left)
	if (bSteeringStabilizerActiveLeft == false)
	{
		LeftTrack.DriveTorque = LeftTrack.TorqueTransfer * DriveTorque;
		LeftTrack.DriveForce = UpdatedMesh->GetForwardVector() * (LeftTrackTorque / SprocketRadius);
	}
	else
	{
		LeftTrack.DriveTorque = 0.f;
		LeftTrack.DriveForce = FVector::ZeroVector;
	}
}

void UPrvVehicleMovementComponent::UpdateAntiRollover(float DeltaTime)
{
	const FVector VehicleZ = UpdatedMesh->GetUpVector();
	const FVector WorldZ = FVector::UpVector;
	const FVector AntiRolloverVector = FVector::CrossProduct(VehicleZ, WorldZ);
	const float Sine = AntiRolloverVector.Size();

	if (Sine > LastAntiRolloverValue || Sine >= AntiRolloverValueThreshold)
	{
		const float TorqueMultiplier = AntiRolloverForceCurve.GetRichCurve()->Eval(Sine);
		UpdatedMesh->AddTorqueInRadians(AntiRolloverVector * TorqueMultiplier);
	}

	LastAntiRolloverValue = Sine;
}

void UPrvVehicleMovementComponent::UpdateSuspension(float DeltaTime)
{
	PRV_CYCLE_COUNTER(STAT_PrvMovementUpdateSuspension);

	// Limit delta time to prevent teleporting vehicles on lag (too much velocity per frame can be applied in this case)
	static float MaxDeltaTime = 1.f / 15.f;
	if (DeltaTime > MaxDeltaTime)
	{
		UE_LOG(LogPrvVehicle, Warning, TEXT("DeltaTime is too big: %f, clamp now to: %f"), DeltaTime, MaxDeltaTime);
		DeltaTime = MaxDeltaTime;
	}

	// Refresh friction points counter
	const int32 ActiveWheelsNum = ActiveFrictionPoints;
	ActiveFrictionPoints = 0;
	ActiveDrivenFrictionPoints = 0;

	TArray<AActor*> IgnoredActors;
	const EDrawDebugTrace::Type DebugType = IsDebug() ? EDrawDebugTrace::ForOneFrame : EDrawDebugTrace::None;

	const bool bUseLineTrace = UseLineTrace();

	for (auto& SuspState : SuspensionData)
	{
		const FVector SuspUpVector = UpdatedMesh->GetComponentTransform().TransformVectorNoScale(UKismetMathLibrary::GetUpVector(SuspState.SuspensionInfo.Rotation));
		const FVector SuspWorldLocation = UpdatedMesh->GetComponentTransform().TransformPosition(SuspState.SuspensionInfo.Location);
		const FVector SuspTraceEndLocation = SuspWorldLocation - SuspUpVector * (SuspState.SuspensionInfo.Length + SuspState.SuspensionInfo.MaxDrop);
		const FVector RadiusUpVector = SuspUpVector * SuspState.SuspensionInfo.CollisionRadius;

		// Make trace to touch the ground
		FHitResult Hit;
		bool bHit = false;
		bool bHitValid = false;

		// For cylindrical wheels only
		if (FMath::Abs(DefaultCollisionWidth) > SMALL_NUMBER && !bUseLineTrace)
		{
			TArray<FHitResult> Hits;

#if ENGINE_MINOR_VERSION >= 15 || ENGINE_MAJOR_VERSION >= 5
			bHit = UKismetSystemLibrary::SphereTraceMulti(this, SuspWorldLocation, SuspTraceEndLocation, SuspState.SuspensionInfo.CollisionRadius, SuspensionTraceTypeQuery, bTraceComplex, IgnoredActors, DebugType, Hits, true);
#else
			bHit = UKismetSystemLibrary::SphereTraceMulti_NEW(this, SuspWorldLocation, SuspTraceEndLocation, SuspState.SuspensionInfo.CollisionRadius, SuspensionTraceTypeQuery, bTraceComplex, IgnoredActors, DebugType, Hits, true);
#endif

			// Process hits and find the best one
			float BestDistanceSquared = MAX_FLT;
			for (auto MyHit : Hits)
			{
				// Ignore overlap
				if (!MyHit.bBlockingHit)
				{
					continue;
				}

				FVector HitLocation_SuspSpace = FVector::ZeroVector;

				// Check that it was penetration hit
				if (MyHit.bStartPenetrating)
				{
					HitLocation_SuspSpace = (MyHit.PenetrationDepth - SuspState.SuspensionInfo.CollisionRadius) * UpdatedMesh->GetComponentTransform().InverseTransformVectorNoScale(MyHit.Normal);
				}
				else
				{
					// Transform into wheel space
					HitLocation_SuspSpace = UpdatedMesh->GetComponentTransform().InverseTransformPosition(MyHit.ImpactPoint) - SuspState.SuspensionInfo.Location;
				}

				// Apply reverse wheel rotation
				HitLocation_SuspSpace = SuspState.SuspensionInfo.Rotation.UnrotateVector(HitLocation_SuspSpace);

				// Check that is outside the cylinder
				if (FMath::Abs(HitLocation_SuspSpace.Y) < (SuspState.SuspensionInfo.CollisionWidth / 2.f))
				{
					// Select the nearest one
					if (HitLocation_SuspSpace.SizeSquared() < BestDistanceSquared)
					{
						BestDistanceSquared = HitLocation_SuspSpace.SizeSquared();

						Hit = MyHit;
						bHitValid = true;
					}
				}

				// Debug hit points
				if (bShowDebug)
				{
					DrawDebugPoint(GetWorld(), UpdatedMesh->GetComponentTransform().TransformPosition(SuspState.SuspensionInfo.Location + SuspState.SuspensionInfo.Rotation.RotateVector(HitLocation_SuspSpace)), 5.f, FColor::Green, false, /*LifeTime*/ 0.f);
				}
			}
		}
		else
		{
			if (bUseLineTrace)
			{
#if ENGINE_MINOR_VERSION >= 15 || ENGINE_MAJOR_VERSION >= 5
				bHit = UKismetSystemLibrary::LineTraceSingle(this, SuspWorldLocation + RadiusUpVector, SuspTraceEndLocation - RadiusUpVector, SuspensionTraceTypeQuery, bTraceComplex, IgnoredActors, DebugType, Hit, true);
#else
				bHit = UKismetSystemLibrary::LineTraceSingle_NEW(this, SuspWorldLocation + RadiusUpVector, SuspTraceEndLocation - RadiusUpVector, SuspensionTraceTypeQuery, bTraceComplex, IgnoredActors, DebugType, Hit, true);
#endif
			}
			else
			{
#if ENGINE_MINOR_VERSION >= 15 || ENGINE_MAJOR_VERSION >= 5
				bHit = UKismetSystemLibrary::SphereTraceSingle(this, SuspWorldLocation, SuspTraceEndLocation, SuspState.SuspensionInfo.CollisionRadius, SuspensionTraceTypeQuery, bTraceComplex, IgnoredActors, DebugType, Hit, true);
#else
				bHit = UKismetSystemLibrary::SphereTraceSingle_NEW(this, SuspWorldLocation, SuspTraceEndLocation, SuspState.SuspensionInfo.CollisionRadius, SuspensionTraceTypeQuery, bTraceComplex, IgnoredActors, DebugType, Hit, true);
#endif
			}

			bHitValid = bHit;
		}

		// Conver line hit to "sphere" hit
		if (bUseLineTrace && bHitValid)
		{
			Hit.Location = Hit.ImpactPoint + RadiusUpVector;
			Hit.Distance = (Hit.Location - SuspWorldLocation).Size();
		}

		// Additional check that hit is valid (for non-spherical wheel)
		if (bHitValid)
		{
			// Transform impact point to actor space
			const FVector HitActorLocation = UpdatedMesh->GetComponentTransform().InverseTransformPosition(Hit.ImpactPoint);

			// Check that collision is under suspension
			if (HitActorLocation.Z >= SuspState.SuspensionInfo.Location.Z)
			{
				if (bDebugSuspensionLimits)
				{
					UE_LOG(LogPrvVehicle, Warning, TEXT("Susp Hit Forced to Zero: Collision.Z: %f, Suspension.Z: %f"), HitActorLocation.Z, SuspState.SuspensionInfo.Location.Z);
				}

				// Force maximum compression
				Hit.ImpactPoint = SuspWorldLocation;
				Hit.ImpactNormal = SuspUpVector;
				Hit.Distance = 0.f;
			}
		}

		// Process hit results
		if (bHitValid)
		{
			// Clamp suspension length because MaxDrop distance is for visuals only (non-effective compression)
			const float NewSuspensionLength = FMath::Clamp(Hit.Distance, 0.f, SuspState.SuspensionInfo.Length);

			const float SpringCompressionRatio = FMath::Clamp((SuspState.SuspensionInfo.Length - NewSuspensionLength) / SuspState.SuspensionInfo.Length, 0.f, 1.f);
			const float TargetVelocity = 0.f; // @todo Target velocity can be different for wheeled vehicles

			// Original suspension velocity
			const float DiscreteSuspensionVelocity = (NewSuspensionLength - SuspState.PreviousLength) / DeltaTime;

			float SuspensionForce = 0.f;

			// Compression and decompression have different suspension quality
			float SuspensionDamping = 0.f;
			const float SuspensionStiffness = SuspState.SuspensionInfo.Stiffness * StiffnessFactor;

			if (DiscreteSuspensionVelocity < 0)
			{
				SuspensionDamping = SuspState.SuspensionInfo.CompressionDamping * CompressionDampingFactor;
			}
			else
			{
				SuspensionDamping = SuspState.SuspensionInfo.DecompressionDamping * DecompressionDampingFactor;
			}

			// Check we should correct the damping
			float SuspensionVelocity = DiscreteSuspensionVelocity;
			if (bCustomDampingCorrection && FMath::Abs(DampingCorrectionFactor) > SMALL_NUMBER && FMath::Abs(DiscreteSuspensionVelocity) > SMALL_NUMBER)
			{
				// Suspension velocity damping (because it works not discrete for DeltaTime)
				const float suspVel = DiscreteSuspensionVelocity / 100.f;
				const float k = SuspensionStiffness / 100.f;
				const float D = SuspensionDamping / 100.f;
				const float m = UpdatedMesh->GetMass();		   // VehicleMass
				const float b = SuspensionDamping / (2.f * m); // DampingCoefficient
				const float a_lin = FMath::Square(b) - (k / m);
				const float a = FMath::Sqrt(FMath::Max(1.f, a_lin)); // FrictionCoefficient
				const float A = suspVel / (2.f * a);				 // InitialDampingEffect
				const float B = -A;
				const float dL_old = suspVel * DeltaTime;
				const float dL_new = FMath::Exp(-b * DeltaTime) * (A * FMath::Exp(a * DeltaTime) + B * FMath::Exp(-a * DeltaTime));
				const float Kl = dL_new / dL_old;
				SuspensionVelocity = suspVel * FMath::Pow(Kl, DampingCorrectionFactor);

				if (bDebugDampingCorrection)
				{
					if (a_lin < 1.f)
					{
						UE_LOG(LogPrvVehicle, Error, TEXT("a_lin is too small: %f"), a_lin);
					}

					UE_LOG(LogPrvVehicle, Warning, TEXT("DeltaTime: %f, suspVel: %f, k: %f, m: %f, D: %f, a: %f, b: %f, k/m: %f, A: %f, dL_old: %f, dL_new: %f, suspVelCorrected: %f"),
						DeltaTime, suspVel, k, m, D, a, b, (k / m), A, dL_old, dL_new, SuspensionVelocity);
				}
			}

			// Adaptive damping correction
			if (bAdaptiveDampingCorrection)
			{
				const float D = SuspensionDamping / 100.f;
				const float m = UpdatedMesh->GetMass(); // VehicleMass

				const float AdaptiveExp = (1 - FMath::Exp((-D) * ActiveWheelsNum / m * DeltaTime));
				if (FMath::Abs(AdaptiveExp) > SMALL_NUMBER)
				{
					const float AdaptiveSuspensionDamping = AdaptiveExp * m / (ActiveWheelsNum * DeltaTime);

					if (bDebugDampingCorrection)
					{
						UE_LOG(LogPrvVehicle, Warning, TEXT("SuspensionDamping: %f, AdaptiveSuspensionDamping: %f, ActiveWheelsNum: %d"),
							SuspensionDamping, (AdaptiveSuspensionDamping * 100.f), ActiveWheelsNum);
					}

					SuspensionDamping = AdaptiveSuspensionDamping * 100.f;
				}
				else if (bDebugDampingCorrection)
				{
					UE_LOG(LogPrvVehicle, Warning, TEXT("SuspensionDamping: %f, AdaptiveExp: 0"), SuspensionDamping);
				}
			}

			// Apply suspension force
			SuspensionForce = (TargetVelocity - SuspensionVelocity) * SuspensionDamping + SpringCompressionRatio * SuspensionStiffness;

			if (SuspensionForce < 0.f)
			{
				if (bClampSuspensionForce)
				{
					SuspensionForce = 0.f;
				}
				else
				{
					UE_LOG(LogPrvVehicle, Warning, TEXT("Negative SuspensionForce = %f"), SuspensionForce);
				}
			}

			const FVector SuspensionDirection = (bWheeledVehicle) ? Hit.ImpactNormal : SuspUpVector;
			SuspState.SuspensionForce = SuspensionForce * SuspensionDirection;

			SuspState.WheelCollisionLocation = Hit.ImpactPoint;
			SuspState.WheelCollisionNormal = Hit.ImpactNormal;
			SuspState.PreviousLength = NewSuspensionLength;
			SuspState.WheelTouchedGround = true;
			SuspState.SurfaceType = UGameplayStatics::GetSurfaceType(Hit);

			if (SuspState.VisualLength < Hit.Distance)
			{
				SuspState.VisualLength = FMath::Lerp(SuspState.VisualLength, Hit.Distance, FMath::Clamp(DeltaTime * DropFactor, 0.f, 1.f));
			}
			else
			{
				SuspState.VisualLength = Hit.Distance;
			}

			// Current wheel touches ground
			ActiveFrictionPoints++;

			// Active driving wheels are calculated separately (has sense for cars only)
			if (!bWheeledVehicle || SuspState.SuspensionInfo.bDrivingWheel)
			{
				ActiveDrivenFrictionPoints++;
			}
		}
		else
		{
			// If there is no collision then suspension is relaxed
			SuspState.SuspensionForce = FVector::ZeroVector;
			SuspState.WheelCollisionLocation = FVector::ZeroVector;
			SuspState.WheelCollisionNormal = FVector::UpVector;
			SuspState.PreviousLength = SuspState.SuspensionInfo.Length;
			SuspState.VisualLength = FMath::Lerp(SuspState.VisualLength, SuspState.SuspensionInfo.Length + SuspState.SuspensionInfo.MaxDrop, FMath::Clamp(DeltaTime * DropFactor, 0.f, 1.f)); // @todo Make it non-momental
			SuspState.WheelTouchedGround = false;
			SuspState.SurfaceType = EPhysicalSurface::SurfaceType_Default;
		}

		// Add suspension force if spring compressed
		if (ShouldAddForce() && !SuspState.SuspensionForce.IsZero())
		{
			UpdatedMesh->AddForceAtLocation(SuspState.SuspensionForce, SuspWorldLocation);
		}

		// Push suspension force to environment
		if (bHit)
		{
			UPrimitiveComponent* PrimitiveComponent = Hit.Component.Get();
			if (PrimitiveComponent)
			{
				// Generate hit event
				if (bNotifyRigidBodyCollision)
				{
					UpdatedMesh->DispatchBlockingHit(*GetOwner(), Hit);
				}

				// Push the force
				if (PrimitiveComponent->IsSimulatingPhysics())
				{
					PrimitiveComponent->AddForceAtLocation(-SuspState.SuspensionForce, SuspWorldLocation);
				}
			}
		}

		// Debug
		if (bShowDebug)
		{
			// Suspension force
			DrawDebugLine(GetWorld(), SuspWorldLocation, SuspWorldLocation + SuspState.SuspensionForce * 0.0001f, FColor::Green, false, /*LifeTime*/ 0.f, /*DepthPriority*/ 0, /*Thickness*/ 4.f);

			// Suspension length
			DrawDebugPoint(GetWorld(), SuspWorldLocation, 5.f, FColor(200, 0, 230), false, /*LifeTime*/ 0.f);
			DrawDebugLine(GetWorld(), SuspWorldLocation, SuspWorldLocation - SuspUpVector * SuspState.PreviousLength, FColor::Blue, false, 0.f, 0, 4.f);
			DrawDebugLine(GetWorld(), SuspWorldLocation, SuspWorldLocation - SuspUpVector * SuspState.SuspensionInfo.Length, FColor::Red, false, 0.f, 0, 2.f);

			// Draw wheel
			if (bHit && SuspState.SuspensionInfo.CollisionWidth != 0.f)
			{
				FColor WheelColor = bHitValid ? FColor::Cyan : FColor::White;
				FVector LineOffset = UpdatedMesh->GetComponentTransform().GetRotation().RotateVector(FVector(0.f, SuspState.SuspensionInfo.CollisionWidth / 2.f, 0.f));
				LineOffset = SuspState.SuspensionInfo.Rotation.RotateVector(LineOffset);
				DrawDebugCylinder(GetWorld(), Hit.Location - LineOffset, Hit.Location + LineOffset, SuspState.SuspensionInfo.CollisionRadius, 16, WheelColor, false, /*LifeTime*/ 0.f, 100);
			}
		}
	}
}

void UPrvVehicleMovementComponent::UpdateSuspensionVisualsOnly(float DeltaTime)
{
	PRV_CYCLE_COUNTER(STAT_PrvMovementUpdateSuspensionVisualsOnly);

	// Suspension
	if (bShouldAnimateWheels)
	{
		TArray<AActor*> IgnoredActors;
		EDrawDebugTrace::Type DebugType = IsDebug() ? EDrawDebugTrace::ForOneFrame : EDrawDebugTrace::None;

		// For simulated proxy, suspension use line trace
		bool bUseLineTrace = UseLineTrace();

		if (!bUseLineTrace && bSimplifiedSuspensionByCamera)
		{
			FVector RelativeCameraVector;
			FVector RelativeMeshForwardVector;
			if (GetCameraVector(RelativeCameraVector, RelativeMeshForwardVector))
			{
				RelativeCameraVector.Z = 0;
				if (RelativeCameraVector.SizeSquared() > SMALL_NUMBER)
				{
					RelativeCameraVector.Normalize();
					if (FMath::Abs(RelativeCameraVector | RelativeMeshForwardVector) > 0.9f)
					{
						bUseLineTrace = true;
					}
				}
			}
		}

		for (auto& SuspState : SuspensionData)
		{
			const FVector SuspUpVector = UpdatedMesh->GetComponentTransform().TransformVectorNoScale(UKismetMathLibrary::GetUpVector(SuspState.SuspensionInfo.Rotation));
			const FVector SuspWorldLocation = UpdatedMesh->GetComponentTransform().TransformPosition(SuspState.SuspensionInfo.Location);
			const FVector SuspTraceEndLocation = SuspWorldLocation - SuspUpVector * (SuspState.SuspensionInfo.Length + SuspState.SuspensionInfo.MaxDrop);
			const FVector RadiusUpVector = SuspUpVector * SuspState.SuspensionInfo.CollisionRadius;

			// Make trace to touch the ground
			FHitResult Hit;
			bool bHit = false;
			bool bHitValid = false;

			// For cylindrical wheels only
			if (FMath::Abs(DefaultCollisionWidth) > SMALL_NUMBER && !bUseLineTrace)
			{
				TArray<FHitResult> Hits;

#if ENGINE_MINOR_VERSION >= 15 || ENGINE_MAJOR_VERSION >= 5
				bHit = UKismetSystemLibrary::SphereTraceMulti(this, SuspWorldLocation, SuspTraceEndLocation, SuspState.SuspensionInfo.CollisionRadius, SuspensionTraceTypeQuery, bTraceComplex, IgnoredActors, DebugType, Hits, true);
#else
				bHit = UKismetSystemLibrary::SphereTraceMulti_NEW(this, SuspWorldLocation, SuspTraceEndLocation, SuspState.SuspensionInfo.CollisionRadius, SuspensionTraceTypeQuery, bTraceComplex, IgnoredActors, DebugType, Hits, true);
#endif

				// Process hits and find the best one
				float BestDistanceSquared = MAX_FLT;
				for (auto MyHit : Hits)
				{
					// Ignore overlap
					if (!MyHit.bBlockingHit)
					{
						continue;
					}

					FVector HitLocation_SuspSpace = FVector::ZeroVector;

					// Check that it was penetration hit
					if (MyHit.bStartPenetrating)
					{
						HitLocation_SuspSpace = (MyHit.PenetrationDepth - SuspState.SuspensionInfo.CollisionRadius) * UpdatedMesh->GetComponentTransform().InverseTransformVectorNoScale(MyHit.Normal);
					}
					else
					{
						// Transform into wheel space
						HitLocation_SuspSpace = UpdatedMesh->GetComponentTransform().InverseTransformPosition(MyHit.ImpactPoint) - SuspState.SuspensionInfo.Location;
					}

					// Apply reverse wheel rotation
					HitLocation_SuspSpace = SuspState.SuspensionInfo.Rotation.UnrotateVector(HitLocation_SuspSpace);

					// Check that is outside the cylinder
					if (FMath::Abs(HitLocation_SuspSpace.Y) < (SuspState.SuspensionInfo.CollisionWidth / 2.f))
					{
						// Select the nearest one
						if (HitLocation_SuspSpace.SizeSquared() < BestDistanceSquared)
						{
							BestDistanceSquared = HitLocation_SuspSpace.SizeSquared();

							Hit = MyHit;
							bHitValid = true;
						}
					}

					// Debug hit points
					if (bShowDebug)
					{
						DrawDebugPoint(GetWorld(), UpdatedMesh->GetComponentTransform().TransformPosition(SuspState.SuspensionInfo.Location + SuspState.SuspensionInfo.Rotation.RotateVector(HitLocation_SuspSpace)), 5.f, FColor::Green, false, /*LifeTime*/ 0.f);
					}
				}
			}
			else
			{
				if (bUseLineTrace)
				{
#if ENGINE_MINOR_VERSION >= 15 || ENGINE_MAJOR_VERSION >= 5
					bHit = UKismetSystemLibrary::LineTraceSingle(this, SuspWorldLocation + RadiusUpVector, SuspTraceEndLocation - RadiusUpVector, SuspensionTraceTypeQuery, bTraceComplex, IgnoredActors, DebugType, Hit, true);
#else
					bHit = UKismetSystemLibrary::LineTraceSingle_NEW(this, SuspWorldLocation + RadiusUpVector, SuspTraceEndLocation - RadiusUpVector, SuspensionTraceTypeQuery, bTraceComplex, IgnoredActors, DebugType, Hit, true);
#endif
				}
				else
				{
#if ENGINE_MINOR_VERSION >= 15 || ENGINE_MAJOR_VERSION >= 5
					bHit = UKismetSystemLibrary::SphereTraceSingle(this, SuspWorldLocation, SuspTraceEndLocation, SuspState.SuspensionInfo.CollisionRadius, SuspensionTraceTypeQuery, bTraceComplex, IgnoredActors, DebugType, Hit, true);
#else
					bHit = UKismetSystemLibrary::SphereTraceSingle_NEW(this, SuspWorldLocation, SuspTraceEndLocation, SuspState.SuspensionInfo.CollisionRadius, SuspensionTraceTypeQuery, bTraceComplex, IgnoredActors, DebugType, Hit, true);
#endif
				}

				bHitValid = bHit;
			}

			// Conver line hit to "sphere" hit
			if (bUseLineTrace && bHitValid)
			{
				Hit.Location = Hit.ImpactPoint + RadiusUpVector;
				Hit.Distance = (Hit.Location - SuspWorldLocation).Size();
			}

			// Additional check that hit is valid (for non-spherical wheel)
			if (bHitValid)
			{
				// Transform impact point to actor space
				const FVector HitActorLocation = UpdatedMesh->GetComponentTransform().InverseTransformPosition(Hit.ImpactPoint);

				// Check that collision is under suspension
				if (HitActorLocation.Z >= SuspState.SuspensionInfo.Location.Z)
				{
					if (bDebugSuspensionLimits)
					{
						UE_LOG(LogPrvVehicle, Warning, TEXT("Susp Hit Forced to Zero: Collision.Z: %f, Suspension.Z: %f"), HitActorLocation.Z, SuspState.SuspensionInfo.Location.Z);
					}

					// Force maximum compression
					Hit.ImpactPoint = SuspWorldLocation;
					Hit.ImpactNormal = SuspUpVector;
					Hit.Distance = 0.f;
				}
			}

			// Process hit results
			if (bHitValid)
			{
				// Clamp suspension length because MaxDrop distance is for visuals only (non-effective compression)
				const float NewSuspensionLength = FMath::Clamp(Hit.Distance, 0.f, SuspState.SuspensionInfo.Length);

				const float SpringCompressionRatio = FMath::Clamp((SuspState.SuspensionInfo.Length - NewSuspensionLength) / SuspState.SuspensionInfo.Length, 0.f, 1.f);

				SuspState.WheelCollisionLocation = Hit.ImpactPoint;
				SuspState.WheelCollisionNormal = Hit.ImpactNormal;
				SuspState.PreviousLength = NewSuspensionLength;
				SuspState.WheelTouchedGround = true;
				SuspState.SurfaceType = UGameplayStatics::GetSurfaceType(Hit);

				if (SuspState.VisualLength < Hit.Distance)
				{
					SuspState.VisualLength = FMath::Lerp(SuspState.VisualLength, Hit.Distance, FMath::Clamp(DeltaTime * DropFactor, 0.f, 1.f));
				}
				else
				{
					SuspState.VisualLength = Hit.Distance;
				}
			}
			else
			{
				// If there is no collision then suspension is relaxed
				SuspState.SuspensionForce = FVector::ZeroVector;
				SuspState.WheelCollisionLocation = FVector::ZeroVector;
				SuspState.WheelCollisionNormal = FVector::UpVector;
				SuspState.PreviousLength = SuspState.SuspensionInfo.Length;
				SuspState.VisualLength = FMath::Lerp(SuspState.VisualLength, SuspState.SuspensionInfo.Length + SuspState.SuspensionInfo.MaxDrop, FMath::Clamp(DeltaTime * DropFactor, 0.f, 1.f));
				SuspState.WheelTouchedGround = false;
				SuspState.SurfaceType = EPhysicalSurface::SurfaceType_Default;
			}

			// @todo Possible push some suspension force to environment

			// Debug
			if (bShowDebug)
			{
				// Suspension length
				DrawDebugPoint(GetWorld(), SuspWorldLocation, 5.f, FColor(200, 0, 230), false, /*LifeTime*/ 0.f);
				DrawDebugLine(GetWorld(), SuspWorldLocation, SuspWorldLocation - SuspUpVector * SuspState.PreviousLength, FColor::Blue, false, 0.f, 0, 4.f);
				DrawDebugLine(GetWorld(), SuspWorldLocation, SuspWorldLocation - SuspUpVector * SuspState.SuspensionInfo.Length, FColor::Red, false, 0.f, 0, 2.f);

				// Draw wheel
				if (bHit && SuspState.SuspensionInfo.CollisionWidth != 0.f)
				{
					FColor WheelColor = bHitValid ? FColor::Cyan : FColor::White;
					FVector LineOffset = UpdatedMesh->GetComponentTransform().GetRotation().RotateVector(FVector(0.f, SuspState.SuspensionInfo.CollisionWidth / 2.f, 0.f));
					LineOffset = SuspState.SuspensionInfo.Rotation.RotateVector(LineOffset);
					DrawDebugCylinder(GetWorld(), Hit.Location - LineOffset, Hit.Location + LineOffset, SuspState.SuspensionInfo.CollisionRadius, 16, WheelColor, false, /*LifeTime*/ 0.f, 100);
				}
			}
		}
	}

	// -- [Car] --
	if (bWheeledVehicle)
	{
		// Update driving wheels for wheeled vehicles
		for (auto& SuspState : SuspensionData)
		{
			if (SuspState.SuspensionInfo.bSteeringWheel)
			{
				SuspState.SuspensionInfo.Rotation.Yaw = FMath::Lerp(SuspState.SuspensionInfo.Rotation.Yaw, EffectiveSteeringAngularSpeed, DeltaTime * (SteeringUpRatio + SteeringDownRatio) / 2.f);
			}
		}
	}
}

void UPrvVehicleMovementComponent::UpdateFriction(float DeltaTime)
{
	PRV_CYCLE_COUNTER(STAT_PrvMovementUpdateFriction);

	// Reset tracks friction
	RightTrack.KineticFrictionTorque = 0.f;
	RightTrack.RollingFrictionTorque = 0.f;
	LeftTrack.KineticFrictionTorque = 0.f;
	LeftTrack.RollingFrictionTorque = 0.f;

	float MinimumWheelAngularSpeedLeft = BIG_NUMBER;
	float MinimumWheelAngularSpeedRight = BIG_NUMBER;

	// Process suspension
	for (auto& SuspState : SuspensionData)
	{
		if (SuspState.WheelTouchedGround)
		{
			// Cache current track info
			FTrackInfo* WheelTrack = (SuspState.SuspensionInfo.bRightTrack) ? &RightTrack : &LeftTrack;
			float& MinimumWheelAngularSpeed = (SuspState.SuspensionInfo.bRightTrack) ? MinimumWheelAngularSpeedLeft : MinimumWheelAngularSpeedRight;

			/////////////////////////////////////////////////////////////////////////
			// Drive force

			// Calculate wheel load
			SuspState.WheelLoad = UKismetMathLibrary::ProjectVectorOnToVector(SuspState.SuspensionForce, SuspState.WheelCollisionNormal).Size();

			// Wheel forward vector
			const FVector WheelDirection = SuspState.SuspensionInfo.Rotation.RotateVector(UpdatedMesh->GetForwardVector());

			// Get Velocity at location
			FVector WorldPointVelocity = FVector::ZeroVector;
			if (bUseCustomVelocityCalculations)
			{
				const FVector PlaneLocalVelocity = GetOwner()->GetTransform().InverseTransformVectorNoScale(UpdatedMesh->GetPhysicsLinearVelocity());
				const FVector PlaneAngularVelocity = GetOwner()->GetTransform().InverseTransformVectorNoScale(UpdatedMesh->GetPhysicsAngularVelocityInDegrees());
				const FVector LocalCOM = GetOwner()->GetTransform().InverseTransformPosition(UpdatedMesh->GetCenterOfMass());
				const FVector LocalCollisionLocation = GetOwner()->GetTransform().InverseTransformPosition(SuspState.WheelCollisionLocation);
				const FVector LocalPointVelocity = PlaneLocalVelocity + FVector::CrossProduct(FMath::DegreesToRadians(PlaneAngularVelocity), (LocalCollisionLocation - LocalCOM));
				WorldPointVelocity = GetOwner()->GetTransform().TransformVectorNoScale(LocalPointVelocity);
			}
			else
			{
				WorldPointVelocity = UpdatedMesh->GetPhysicsLinearVelocityAtPoint(SuspState.WheelCollisionLocation);
			}

			// Calculate wheel velocity relative to track (with simple Kalman filter)
			const FVector WheelCollisionVelocity = (WorldPointVelocity + SuspState.PreviousWheelCollisionVelocity) / 2.f;

			// Cache last velocity
			SuspState.PreviousWheelCollisionVelocity = WheelCollisionVelocity;

			// Apply linear friction
			FVector WheelVelocity = FVector::ZeroVector - WheelCollisionVelocity;

			// Add driving force
			if (!bWheeledVehicle || SuspState.SuspensionInfo.bDrivingWheel)
			{
				WheelVelocity += (WheelDirection * WheelTrack->LinearSpeed);
			}

			const FVector RelativeWheelVelocity = UKismetMathLibrary::ProjectVectorOnToPlane(WheelVelocity, SuspState.WheelCollisionNormal);

			// Get friction coefficients
			const float MuStatic = CalculateFrictionCoefficient(RelativeWheelVelocity, WheelDirection, StaticFrictionCoefficientEllipse);
			const float MuKinetic = CalculateFrictionCoefficient(RelativeWheelVelocity, WheelDirection, KineticFrictionCoefficientEllipse);

			// Mass and friction forces
			const float VehicleMass = UpdatedMesh->GetMass();
			const FVector FrictionXVector = UKismetMathLibrary::ProjectVectorOnToPlane(UpdatedMesh->GetForwardVector(), SuspState.WheelCollisionNormal).GetSafeNormal();
			const FVector FrictionYVector = UKismetMathLibrary::ProjectVectorOnToPlane(UpdatedMesh->GetRightVector(), SuspState.WheelCollisionNormal).GetSafeNormal();

			// Current wheel force contbution
			FVector WheelBalancedForce = FVector::ZeroVector;
			if (ActiveFrictionPoints != 0)
			{
				const FVector GravityDirection = -FVector::UpVector;
				const FVector GravityBasedFriction = UKismetMathLibrary::ProjectVectorOnToPlane(GravityDirection * UPhysicsSettings::Get()->DefaultGravityZ * VehicleMass / ActiveFrictionPoints, UpdatedMesh->GetUpVector());
				WheelBalancedForce = RelativeWheelVelocity * VehicleMass / DeltaTime / ActiveFrictionPoints + GravityBasedFriction;
			}

			// @temp For non-driving wheels X friction is disabled
			float LongitudeFrictionFactor = 1.f;
			if (bWheeledVehicle && !SuspState.SuspensionInfo.bDrivingWheel)
			{
				LongitudeFrictionFactor = 0.f;
			}

			// Full friction forces
			const FVector StaticFrictionForceX = UKismetMathLibrary::ProjectVectorOnToVector(WheelBalancedForce, FrictionXVector) * StaticFrictionCoefficientEllipse.X * LongitudeFrictionFactor * FMath::Sign(WheelTrack->BrakeRatio);
			const FVector StaticFrictionForceY = UKismetMathLibrary::ProjectVectorOnToVector(WheelBalancedForce, FrictionYVector) * StaticFrictionCoefficientEllipse.Y;
			const FVector FullStaticFrictionForce =
				StaticFrictionForceX.GetClampedToMaxSize(SuspState.WheelLoad * StaticFrictionCoefficientEllipse.X) +
				StaticFrictionForceY.GetClampedToMaxSize(SuspState.WheelLoad * StaticFrictionCoefficientEllipse.Y);

			const FVector KineticFrictionForceX = UKismetMathLibrary::ProjectVectorOnToVector(WheelBalancedForce, FrictionXVector) * KineticFrictionCoefficientEllipse.X * LongitudeFrictionFactor;
			const FVector KineticFrictionForceY = UKismetMathLibrary::ProjectVectorOnToVector(WheelBalancedForce, FrictionYVector) * KineticFrictionCoefficientEllipse.Y;
			const FVector FullKineticFrictionForce = KineticFrictionForceX.GetClampedToMaxSize(SuspState.WheelLoad * StaticFrictionCoefficientEllipse.X) + KineticFrictionForceY.GetClampedToMaxSize(SuspState.WheelLoad * StaticFrictionCoefficientEllipse.Y);

			// Drive Force from transmission torque
			FVector TransmissionDriveForce = UKismetMathLibrary::ProjectVectorOnToPlane(WheelTrack->DriveForce, SuspState.WheelCollisionNormal);

			if (bScaleForceToActiveFrictionPoints && ActiveDrivenFrictionPoints != 0 && SuspensionData.Num() != 0)
			{
				const float Ratio = static_cast<float>(SuspensionData.Num()) / static_cast<float>(ActiveDrivenFrictionPoints);
				TransmissionDriveForce *= Ratio;
			}

			// Full drive forces
			const FVector FullStaticDriveForce = TransmissionDriveForce * StaticFrictionCoefficientEllipse.X * LongitudeFrictionFactor;
			const FVector FullKineticDriveForce = TransmissionDriveForce * KineticFrictionCoefficientEllipse.X * LongitudeFrictionFactor;

			// Full forces
			const FVector FullStaticForce = FullStaticDriveForce + FullStaticFrictionForce;
			const FVector FullKineticForce = FullKineticDriveForce + FullKineticFrictionForce;

			// We want to apply higher friction if forces are bellow static friction limit
			bUseKineticFriction = FullStaticDriveForce.Size() >= (SuspState.WheelLoad * MuStatic);
			const FVector FullKineticFrictionNormalizedForce = bUseKineticFriction ? FullKineticFrictionForce.GetSafeNormal() : FVector::ZeroVector;
			const FVector ApplicationForce = bUseKineticFriction
												 ? FullKineticForce.GetClampedToMaxSize(SuspState.WheelLoad * MuKinetic)
												 : FullStaticForce.GetClampedToMaxSize(SuspState.WheelLoad * MuStatic);

			if (bUseKineticFriction == false)
			{
				const float WorldPointForwardVectorSpeed = FVector::DotProduct(WorldPointVelocity, UpdatedMesh->GetForwardVector());
				const float CurrentAngularSpeed = WorldPointForwardVectorSpeed / SprocketRadius;
				MinimumWheelAngularSpeed = FMath::Min(MinimumWheelAngularSpeed, CurrentAngularSpeed);
				WheelTrack->AngularSpeed = MinimumWheelAngularSpeed;
			}

			// Apply force to mesh
			if (ShouldAddForce())
			{
				UpdatedMesh->AddForceAtLocation(ApplicationForce, SuspState.WheelCollisionLocation);
			}

			/////////////////////////////////////////////////////////////////////////
			// Friction torque

			// Friction should work agains real movement
			float FrictionDirectionMultiplier = FMath::Sign(WheelTrack->AngularSpeed) * FMath::Sign(WheelTrack->TorqueTransfer) * ((bReverseGear) ? (-1.f) : 1.f);
			if (FMath::Abs(FrictionDirectionMultiplier) < SMALL_NUMBER)
				FrictionDirectionMultiplier = 1.f;

			// How much of friction force will effect transmission
			FVector TransmissionFrictionForce = FVector::ZeroVector;
			if (bUseKineticFriction && FullKineticFrictionNormalizedForce.SizeSquared() > SMALL_NUMBER)
			{
				TransmissionFrictionForce = UKismetMathLibrary::ProjectVectorOnToVector(ApplicationForce, FullKineticFrictionNormalizedForce) * (-1.f) * (TrackMass + SprocketMass) / VehicleMass * FrictionDirectionMultiplier;
			}
			const FVector WorldFrictionForce = UpdatedMesh->GetComponentTransform().InverseTransformVectorNoScale(TransmissionFrictionForce);
			const float TrackKineticFrictionTorque = UKismetMathLibrary::ProjectVectorOnToVector(WorldFrictionForce, FVector::ForwardVector).X * SprocketRadius;

			WheelTrack->KineticFrictionTorque += (TrackKineticFrictionTorque * KineticFrictionTorqueCoefficient);

			/////////////////////////////////////////////////////////////////////////
			// Rolling friction torque

			// @todo Make this a force instead of torque!
			const float ReverseVelocitySign = (-1.f) * FMath::Sign(WheelTrack->LinearSpeed);
			const float TrackRollingFrictionTorque = SuspState.WheelLoad * RollingFrictionCoefficient * ReverseVelocitySign +
													 SuspState.WheelLoad * FMath::Pow(WheelTrack->LinearSpeed, LinearSpeedPower) * FMath::Pow(RollingVelocityCoefficientSquared, 2.f) * ReverseVelocitySign;

			// Add torque to track
			WheelTrack->RollingFrictionTorque += TrackRollingFrictionTorque;

			/////////////////////////////////////////////////////////////////////////
			// Debug

			if (bShowDebug)
			{
				// Friction type
				if (bUseKineticFriction)
				{
					DrawDebugString(GetWorld(), SuspState.WheelCollisionLocation, TEXT("Kinetic"), nullptr, FColor::Blue, 0.f);
				}
				else
				{
					DrawDebugString(GetWorld(), SuspState.WheelCollisionLocation, TEXT("Static"), nullptr, FColor::Red, 0.f);
				}

				// Force application
				DrawDebugLine(GetWorld(), SuspState.WheelCollisionLocation, SuspState.WheelCollisionLocation + ApplicationForce * 0.0001f, FColor::Cyan, false, 0.f, 0, 10.f);

				// Wheel velocity vectors
				DrawDebugLine(GetWorld(), SuspState.WheelCollisionLocation, SuspState.WheelCollisionLocation + WheelCollisionVelocity, FColor::Yellow, false, 0.f, 0, 8.f);
				DrawDebugLine(GetWorld(), SuspState.WheelCollisionLocation, SuspState.WheelCollisionLocation + RelativeWheelVelocity, FColor::Blue, false, 0.f, 0, 8.f);
			}
		}
		else
		{
			// Reset wheel load
			SuspState.WheelLoad = 0.f;
		}
	}
}

float UPrvVehicleMovementComponent::CalculateFrictionCoefficient(FVector DirectionVelocity, FVector ForwardVector, FVector2D FrictionEllipse)
{
	// dot(A,B)
	const float DirectionDotProduct = FVector::DotProduct(DirectionVelocity.GetSafeNormal(), ForwardVector);

	FVector2D MuVector;
	// x = r1 * dot(A,B)
	MuVector.X = FrictionEllipse.X * DirectionDotProduct;
	// y = r2 * sqrt(1 - dot(A,B)^2 )
	MuVector.Y = FrictionEllipse.Y * FMath::Sqrt(1.f - FMath::Square(DirectionDotProduct));

	return MuVector.Size();
}

void UPrvVehicleMovementComponent::UpdateLinearVelocity(float DeltaTime)
{
	if (ShouldAddForce() && bCustomLinearDamping)
	{
		const FVector LocalLinearVelocity = UpdatedMesh->GetComponentTransform().InverseTransformVectorNoScale(UpdatedMesh->GetPhysicsLinearVelocity());
		const FVector SignVector = FVector(FMath::Sign(LocalLinearVelocity.X), FMath::Sign(LocalLinearVelocity.Y), FMath::Sign(LocalLinearVelocity.Z));
		FVector NewLinearVelocity = LocalLinearVelocity - DeltaTime * (SignVector * DryFrictionLinearDamping + FluidFrictionLinearDamping * LocalLinearVelocity);

		// Clamp to zero vector in per-component basis
		NewLinearVelocity.X = SignVector.X * FMath::Max(0.f, SignVector.X * NewLinearVelocity.X);
		NewLinearVelocity.Y = SignVector.Y * FMath::Max(0.f, SignVector.Y * NewLinearVelocity.Y);
		NewLinearVelocity.Z = SignVector.Z * FMath::Max(0.f, SignVector.Z * NewLinearVelocity.Z);

		UpdatedMesh->SetPhysicsLinearVelocity(UpdatedMesh->GetComponentTransform().TransformVectorNoScale(NewLinearVelocity));

		if (bDebugCustomDamping)
		{
			UE_LOG(LogPrvVehicle, Error, TEXT("Linear damping WAS: %s, NOW: %s"), *LocalLinearVelocity.ToString(), *NewLinearVelocity.ToString());
		}
	}
}

void UPrvVehicleMovementComponent::UpdateAngularVelocity(float DeltaTime)
{
	if (ShouldAddForce() && bCustomAngularDamping)
	{
		const FVector LocalAngularVelocity = UpdatedMesh->GetComponentTransform().InverseTransformVectorNoScale(UpdatedMesh->GetPhysicsAngularVelocityInDegrees());
		const FVector SignVector = FVector(FMath::Sign(LocalAngularVelocity.X), FMath::Sign(LocalAngularVelocity.Y), FMath::Sign(LocalAngularVelocity.Z));
		FVector NewAngularVelocity = LocalAngularVelocity - DeltaTime * (SignVector * DryFrictionAngularDamping + FluidFrictionAngularDamping * LocalAngularVelocity);

		// Clamp to zero vector in per-component basis
		NewAngularVelocity.X = SignVector.X * FMath::Max(0.f, SignVector.X * NewAngularVelocity.X);
		NewAngularVelocity.Y = SignVector.Y * FMath::Max(0.f, SignVector.Y * NewAngularVelocity.Y);
		NewAngularVelocity.Z = SignVector.Z * FMath::Max(0.f, SignVector.Z * NewAngularVelocity.Z);

		UpdatedMesh->SetPhysicsAngularVelocityInDegrees(UpdatedMesh->GetComponentTransform().TransformVectorNoScale(NewAngularVelocity));

		if (bDebugCustomDamping)
		{
			UE_LOG(LogPrvVehicle, Error, TEXT("Angular damping WAS: %s, NOW: %s"), *LocalAngularVelocity.ToString(), *NewAngularVelocity.ToString());
		}
	}
}

void UPrvVehicleMovementComponent::AnimateWheels(float DeltaTime)
{
	for (auto& SuspState : SuspensionData)
	{
		const float EffectiveAngularSpeed = (SuspState.SuspensionInfo.bRightTrack) ? RightTrackEffectiveAngularSpeed : LeftTrackEffectiveAngularSpeed;

		SuspState.RotationAngle -= FMath::RadiansToDegrees(EffectiveAngularSpeed) * DeltaTime * (SprocketRadius / VisualCollisionRadius);
		SuspState.RotationAngle = FRotator::NormalizeAxis(SuspState.RotationAngle);
		SuspState.SteeringAngle = SuspState.SuspensionInfo.Rotation.Yaw;
	}
}

//////////////////////////////////////////////////////////////////////////
// Network

bool UPrvVehicleMovementComponent::ServerUpdateState_Validate(uint16 InQuantizeInput)
{
	return true;
}

void UPrvVehicleMovementComponent::ServerUpdateState_Implementation(uint16 InQuantizeInput)
{
	const int32 QThrottleInput = (int8)(InQuantizeInput & 0xFF);
	const int32 QSteeringInput = ((int8)(((InQuantizeInput >> 8) & 0x7F) << 1)) / 2;
	const int32 QHandbrakeInput = (InQuantizeInput >> 15) & 1;

	SetThrottleInput(QThrottleInput / 127.f);
	SetSteeringInput(QSteeringInput / 63.f);
	bRawHandbrakeInput = QHandbrakeInput;

	LastUserSteeringInput = QSteeringInput;
}

//////////////////////////////////////////////////////////////////////////
// Custom physics handling

bool UPrvVehicleMovementComponent::ConditionalApplyRigidBodyState(FRigidBodyState& UpdatedState, const FOldRigidBodyErrorCorrection& ErrorCorrection, FVector& OutDeltaPos, FName BoneName)
{
	// See UPrimitiveComponent::ConditionalApplyRigidBodyState

	if (UpdatedMesh == nullptr)
	{
		return false;
	}

	bool bUpdated = false;

	// force update if simulation is sleeping on server
	if ((UpdatedState.Flags & ERigidBodyFlags::Sleeping) && UpdatedMesh->RigidBodyIsAwake(BoneName))
	{
		UpdatedState.Flags |= ERigidBodyFlags::NeedsUpdate;
	}

	if (UpdatedState.Flags & ERigidBodyFlags::NeedsUpdate)
	{
		ErrorCorrectionData = ErrorCorrection;
		const bool bRestoredState = ApplyRigidBodyState(UpdatedState, ErrorCorrection, OutDeltaPos, BoneName);
		if (bRestoredState)
		{
			UpdatedState.Flags &= ~ERigidBodyFlags::NeedsUpdate;
		}

		bUpdated = true;
	}

	return bUpdated;
}

bool UPrvVehicleMovementComponent::ApplyRigidBodyState(const FRigidBodyState& NewState, const FOldRigidBodyErrorCorrection& ErrorCorrection, FVector& OutDeltaPos, FName BoneName)
{
	// See UPrimitiveComponent::ApplyRigidBodyState

	if (UpdatedMesh == nullptr)
	{
		return false;
	}

	bool bRestoredState = true;

	FBodyInstance* BI = UpdatedMesh->GetBodyInstance(BoneName);
	if (BI && BI->IsInstanceSimulatingPhysics())
	{
		// failure cases
		const float QuatSizeSqr = NewState.Quaternion.SizeSquared();
		if (QuatSizeSqr < KINDA_SMALL_NUMBER)
		{
			UE_LOG(LogPrvVehicle, Warning, TEXT("Invalid zero quaternion set for body. (%s:%s)"), *GetName(), *BoneName.ToString());
			bCorrectionInProgress = false;
			return bRestoredState;
		}
		else if (FMath::Abs(QuatSizeSqr - 1.f) > KINDA_SMALL_NUMBER)
		{
			UE_LOG(LogPrvVehicle, Warning, TEXT("Quaternion (%f %f %f %f) with non-unit magnitude detected. (%s:%s)"),
				NewState.Quaternion.X, NewState.Quaternion.Y, NewState.Quaternion.Z, NewState.Quaternion.W, *GetName(), *BoneName.ToString());
			bCorrectionInProgress = false;
			return bRestoredState;
		}

		FRigidBodyState CurrentState;
		UpdatedMesh->GetRigidBodyState(CurrentState, BoneName);

		const bool bShouldSleep = (NewState.Flags & ERigidBodyFlags::Sleeping) != 0;

		/////// POSITION CORRECTION ///////

		// Find out how much of a correction we are making
		const FVector DeltaPos = NewState.Position - CurrentState.Position;
		const float DeltaMagSq = DeltaPos.SizeSquared();
		const float BodyLinearSpeedSq = CurrentState.LinVel.SizeSquared();

		// Snap position by default (big correction, or we are moving too slowly)
		FVector UpdatedPos = NewState.Position;
		FVector FixLinVel = FVector::ZeroVector;

		bool bNeedPositionCorrection = false;
		// If its a small correction and velocity is above threshold, only make a partial correction,
		// and calculate a velocity that would fix it over 'fixTime'.
		if (DeltaMagSq < ErrorCorrection.LinearDeltaThresholdSq &&
			BodyLinearSpeedSq >= ErrorCorrection.BodySpeedThresholdSq)
		{
			UpdatedPos = FMath::Lerp(CurrentState.Position, NewState.Position, ErrorCorrection.LinearInterpAlpha);
			FixLinVel = (NewState.Position - UpdatedPos) * ErrorCorrection.LinearRecipFixTime;
			bNeedPositionCorrection = true;
		}

		// Get the linear correction
		OutDeltaPos = UpdatedPos - CurrentState.Position;

		/////// ORIENTATION CORRECTION ///////
		// Get quaternion that takes us from old to new
		const FQuat InvCurrentQuat = CurrentState.Quaternion.Inverse();
		const FQuat DeltaQuat = NewState.Quaternion * InvCurrentQuat;

		FVector DeltaAxis(FVector::ZeroVector);
		float DeltaAng = 0.f; // radians
		DeltaQuat.ToAxisAndAngle(DeltaAxis, DeltaAng);
		DeltaAng = FMath::UnwindRadians(DeltaAng);

		// Snap rotation by default (big correction, or we are moving too slowly)
		FQuat UpdatedQuat = NewState.Quaternion;
		FVector FixAngVel = FVector::ZeroVector; // degrees per second

		bool bNeedOrientationCorrection = false;
		// If the error is small, and we are moving, try to move smoothly to it
		if (FMath::Abs(DeltaAng) < ErrorCorrection.AngularDeltaThreshold)
		{
			UpdatedQuat = FMath::Lerp(CurrentState.Quaternion, NewState.Quaternion, ErrorCorrection.AngularInterpAlpha);
			FixAngVel = DeltaAxis.GetSafeNormal() * FMath::RadiansToDegrees(DeltaAng) * (1.f - ErrorCorrection.AngularInterpAlpha) * ErrorCorrection.AngularRecipFixTime;
			bNeedOrientationCorrection = true;
		}

		if (bNeedPositionCorrection || bNeedOrientationCorrection)
		{
			CorrectionBeganTime = GetWorld()->GetTimeSeconds();
			const float CorrectionTime = FMath::Max(1.f / ErrorCorrection.LinearRecipFixTime, 1.f / ErrorCorrection.AngularRecipFixTime);
			CorrectionEndTime = CorrectionBeganTime + CorrectionTime;
			CorrectionEndState = NewState;
		}

		/////// BODY UPDATE ///////
		BI->SetBodyTransform(FTransform(UpdatedQuat, UpdatedPos), ETeleportType::TeleportPhysics);
		BI->SetLinearVelocity(NewState.LinVel + FixLinVel, false);
		BI->SetAngularVelocityInRadians(FMath::DegreesToRadians(NewState.AngVel + FixAngVel), false);

		// state is restored when no velocity corrections are required
		bRestoredState = (FixLinVel.SizeSquared() < KINDA_SMALL_NUMBER) && (FixAngVel.SizeSquared() < KINDA_SMALL_NUMBER);
		bCorrectionInProgress = !bRestoredState;

		/////// SLEEP UPDATE ///////
		const bool bIsAwake = BI->IsInstanceAwake();
		if (bIsAwake && (bShouldSleep && bRestoredState))
		{
			BI->PutInstanceToSleep();
		}
		else if (!bIsAwake)
		{
			BI->WakeInstance();
		}
	}

	return bRestoredState;
}

//////////////////////////////////////////////////////////////////////////
// Vehicle control

void UPrvVehicleMovementComponent::SetThrottleInput(float Throttle)
{
	LastRawThrottleInput = RawThrottleInput;

	if (!bIsMovementEnabled)
	{
		RawThrottleInput = 0.0f;
		return;
	}

	float NewThrottle = FMath::Clamp(Throttle, -1.0f, 1.0f);

	RawThrottleInputKeep = NewThrottle;

	if (bSteeringStabilizerActiveLeft || bSteeringStabilizerActiveRight)
	{
		NewThrottle = 1.f;
	}

	RawThrottleInput = NewThrottle;
}

void UPrvVehicleMovementComponent::SetSteeringInput(float Steering)
{
	if (!bIsMovementEnabled)
	{
		RawSteeringInput = 0.0f;
		return;
	}

	float NewSteering = FMath::Clamp(Steering, -1.0f, 1.0f);

	RawSteeringInput = NewSteering;
}

void UPrvVehicleMovementComponent::SetHandbrakeInput(bool bNewHandbrake)
{
	bRawHandbrakeInput = bNewHandbrake;
}

void UPrvVehicleMovementComponent::EnableMovement()
{
	bIsMovementEnabled = true;
}

void UPrvVehicleMovementComponent::DisableMovement()
{
	bIsMovementEnabled = false;
	SetSteeringInput(0.0f);
	SetThrottleInput(0.0f);
}

bool UPrvVehicleMovementComponent::IsMoving() const
{
	return bIsMovementEnabled && HasInput();
}

int32 UPrvVehicleMovementComponent::GetLastUserSteeringInput() const
{
	return LastUserSteeringInput;
}

/////////////////////////////////////////////////////////////////////////
// Animation control

void UPrvVehicleMovementComponent::SetWheelsAnimationEnabled(bool bAnimateWheels)
{
	bShouldAnimateWheels = bAnimateWheels;
}

//////////////////////////////////////////////////////////////////////////
// Vehicle stats

float UPrvVehicleMovementComponent::GetForwardSpeed() const
{
	if (UpdatedMesh)
	{
		const float VelocityDirection = FVector::DotProduct(UpdatedMesh->GetForwardVector(), UpdatedMesh->GetComponentVelocity());
		return UpdatedMesh->GetComponentVelocity().Size() * ((VelocityDirection >= 0.f) ? 1.f : -1.f);
	}

	return 0.f;
}

float UPrvVehicleMovementComponent::GetThrottle() const
{
	return ThrottleInput;
}

float UPrvVehicleMovementComponent::GetEngineRotationSpeed() const
{
	return EngineRPM;
}

float UPrvVehicleMovementComponent::GetRawSteeringInput() const
{
	return RawSteeringInput;
}

float UPrvVehicleMovementComponent::GetRawThrottleInput() const
{
	return RawThrottleInputKeep;
}

float UPrvVehicleMovementComponent::GetEngineMaxRotationSpeed() const
{
	return MaxEngineRPM;
}

float UPrvVehicleMovementComponent::GetEngineTorque() const
{
	return EngineTorque;
}

float UPrvVehicleMovementComponent::GetDriveTorqueLeft() const
{
	return LeftTrack.DriveTorque;
}

float UPrvVehicleMovementComponent::GetDriveTorqueRight() const
{
	return RightTrack.DriveTorque;
}

float UPrvVehicleMovementComponent::GetAngularVelocityLeft() const
{
	return LeftTrack.AngularSpeed;
}

float UPrvVehicleMovementComponent::GetAngularVelocityRight() const
{
	return RightTrack.AngularSpeed;
}

float UPrvVehicleMovementComponent::GetBrakeRatioLeft() const
{
	return LeftTrack.BrakeRatio;
}

float UPrvVehicleMovementComponent::GetBrakeRatioRight() const
{
	return RightTrack.BrakeRatio;
}

bool UPrvVehicleMovementComponent::HasTouchGround() const
{
	return (ActiveFrictionPoints > 0);
}

//////////////////////////////////////////////////////////////////////////
// Data access

USkinnedMeshComponent* UPrvVehicleMovementComponent::GetMesh()
{
	return UpdatedMesh;
}

void UPrvVehicleMovementComponent::GetTrackInfoLeft(FTrackInfo& OutTrack) const
{
	OutTrack = LeftTrack;
}

void UPrvVehicleMovementComponent::GetTrackInfoRight(FTrackInfo& OutTrack) const
{
	OutTrack = RightTrack;
}

int32 UPrvVehicleMovementComponent::GetCurrentGear() const
{
	return CurrentGear;
}

int32 UPrvVehicleMovementComponent::GetNeutralGear() const
{
	return NeutralGear;
}

bool UPrvVehicleMovementComponent::IsCurrentGearReverse() const
{
	return (CurrentGear < NeutralGear);
}

FGearInfo UPrvVehicleMovementComponent::GetGearInfo(int32 GearNum) const
{
	// Check that requested gear is valid
	if (GearNum < 0 || GearNum >= GearSetup.Num())
	{
		UE_LOG(LogPrvVehicle, Error, TEXT("Invalid gear index: %d from %d"), GearNum, GearSetup.Num());
		return FGearInfo();
	}

	return GearSetup[GearNum];
}

FGearInfo UPrvVehicleMovementComponent::GetCurrentGearInfo() const
{
	return GetGearInfo(CurrentGear);
}

const TArray<FSuspensionState>& UPrvVehicleMovementComponent::GetSuspensionData() const
{
	return SuspensionData;
}

//////////////////////////////////////////////////////////////////////////
// Effects

void UPrvVehicleMovementComponent::UpdateWheelEffects(float DeltaTime)
{
	PRV_CYCLE_COUNTER(STAT_PrvMovementUpdateWheelEffects);

	if ((GPrvVehicleShowDustEffect != 0) && DustEffect && UpdatedMesh && UpdatedMesh->IsValidLowLevel())
	{
		const float CurrentSpeed = UpdatedMesh->GetComponentVelocity().Size();
		const FRotator MeshRotation = UpdatedMesh->GetComponentRotation();

		// Process suspension
		for (auto& SuspState : SuspensionData)
		{
			if (SuspState.SuspensionInfo.bSpawnDust)
			{
				auto SurfaceType = ForceSurfaceType;
				if (SurfaceType == EPhysicalSurface::SurfaceType_Default)
				{
					SurfaceType = SuspState.SurfaceType;
				}

				// Get vfx corresponding the surface
				UParticleSystem* WheelFX = DustEffect->GetDustFX(SurfaceType, CurrentSpeed);

				// Check current one is active
				const bool bIsVfxActive = SuspState.DustPSC != nullptr && !SuspState.DustPSC->bWasDeactivated && !SuspState.DustPSC->bWasCompleted;

				// Check wheel is touched ground (don't spawn effect if wheels are not animated)
				if (SuspState.WheelTouchedGround && bShouldAnimateWheels)
				{
					UParticleSystem* CurrentFX = SuspState.DustPSC != nullptr ? SuspState.DustPSC->Template : nullptr;

					// Check we need to spawn dust or change the effect
					if (WheelFX != nullptr && (CurrentFX != WheelFX || !bIsVfxActive))
					{
						if (SuspState.DustPSC == nullptr || !SuspState.DustPSC->bWasDeactivated)
						{
							if (SuspState.DustPSC != nullptr)
							{
								SuspState.DustPSC->SetActive(false);
								SuspState.DustPSC->bAutoDestroy = true;
							}

							SuspState.DustPSC = SpawnNewWheelEffect();
						}

						// Update effect location
						if (bUseMeshRotationForEffect)
						{
							SuspState.DustPSC->SetWorldRotation(MeshRotation);
						}
						else
						{
							SuspState.DustPSC->SetRelativeRotation(SuspState.WheelCollisionNormal.Rotation());
						}

						SuspState.DustPSC->SetWorldLocation(SuspState.WheelCollisionLocation);

						// Reactivate effect
						SuspState.DustPSC->SetTemplate(WheelFX);
						SuspState.DustPSC->ActivateSystem();
						SuspState.DustPSC->SetOnlyOwnerSee(GPrvVehicleShowDustEffectForOwnerOnly != 0);
					}
					// Deactivate if no suitable VFX is found for surface type
					else if (WheelFX == nullptr && bIsVfxActive)
					{
						SuspState.DustPSC->SetActive(false);
					}
				}
				// Deactivate particles on ground untouch
				else if (bIsVfxActive)
				{
					SuspState.DustPSC->SetActive(false);
				}

				// Update effect location
				if (bUseMeshRotationForEffect)
				{
					SuspState.DustPSC->SetWorldRotation(MeshRotation);
				}
				else
				{
					SuspState.DustPSC->SetRelativeRotation(SuspState.WheelCollisionNormal.Rotation());
				}

				SuspState.DustPSC->SetWorldLocation(SuspState.WheelCollisionLocation);
			}
		}
	}
}

UParticleSystemComponent* UPrvVehicleMovementComponent::SpawnNewWheelEffect(FName InSocketName, FVector InSocketOffset)
{
	UParticleSystemComponent* DustPSC = NewObject<UParticleSystemComponent>(this);
	DustPSC->bAutoActivate = true;
	DustPSC->bAutoDestroy = false;
	DustPSC->RegisterComponentWithWorld(GetWorld());
	DustPSC->AttachToComponent(UpdatedMesh, FAttachmentTransformRules::SnapToTargetIncludingScale, InSocketName);
	DustPSC->SetRelativeLocation(InSocketOffset);

	return DustPSC;
}

//////////////////////////////////////////////////////////////////////////
// Debug

void UPrvVehicleMovementComponent::DrawDebug(UCanvas* Canvas, float& YL, float& YPos)
{
	// Force draw debug lines
	bShowDebug = true;

	// @todo
}

void UPrvVehicleMovementComponent::DrawDebugLines()
{
	if (bIsSleeping)
	{
		DrawDebugString(GetWorld(), UpdatedMesh->GetCenterOfMass(), TEXT("SLEEP"), nullptr, FColor::Red, 0.f);
	}
	else
	{
		DrawDebugPoint(GetWorld(), UpdatedMesh->GetCenterOfMass(), 25.f, FColor::Yellow, false, /*LifeTime*/ 0.f);
	}
}

//////////////////////////////////////////////////////////////////////////
// Internal data

bool UPrvVehicleMovementComponent::HasInput() const
{
	return FMath::Abs(RawThrottleInput) > SMALL_NUMBER || FMath::Abs(RawSteeringInput) > SMALL_NUMBER || bRawHandbrakeInput;
}

bool UPrvVehicleMovementComponent::ShouldAddForce()
{
	ENetRole OwnerRole = GetOwner()->GetLocalRole();
	const bool bPhysicsIsSimulated = UpdatedComponent ? UpdatedComponent->IsSimulatingPhysics() : false;
	return bPhysicsIsSimulated && ((OwnerRole == ROLE_Authority) || (OwnerRole == ROLE_AutonomousProxy && !bFakeAutonomousProxy));
}

bool UPrvVehicleMovementComponent::UseLineTrace()
{
	ENetRole OwnerRole = GetOwner()->GetLocalRole();
	if (IsRunningDedicatedServer() && OwnerRole == ROLE_Authority)
	{
		if (bSimplifiedSuspension)
		{
			return true;
		}
		else if (bSimplifiedSuspensionWithoutThrottle && FMath::Abs(RawThrottleInput) < SMALL_NUMBER)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return OwnerRole < ROLE_AutonomousProxy;
	}
}

bool UPrvVehicleMovementComponent::GetCameraVector(FVector& RelativeCameraVector, FVector& RelativeMeshForwardVector)
{
	APlayerCameraManager* CameraManager = UGameplayStatics::GetPlayerCameraManager(GetOwner(), 0);
	if (CameraManager == nullptr || UpdatedMesh == nullptr)
	{
		return false;
	}
	FTransform MeshTransform = UpdatedMesh->GetComponentTransform();
	RelativeMeshForwardVector = MeshTransform.InverseTransformVectorNoScale(UpdatedMesh->GetForwardVector());
	RelativeCameraVector = MeshTransform.InverseTransformVectorNoScale(CameraManager->GetCameraRotation().Vector());
	return true;
}

//////////////////////////////////////////////////////////////////////////
// Replication

void UPrvVehicleMovementComponent::UpdateReplicatedCosmeticData()
{
	RepCosmeticData.EngineRPM = static_cast<uint8>((FMath::Min(EngineRPM, MaxEngineRPM) / MaxEngineRPM) * 255.f);
	RepCosmeticData.LeftTrackEffectiveAngularSpeed = static_cast<int8>(FMath::Clamp(FMath::RoundHalfFromZero(LeftTrackEffectiveAngularSpeed), -127.f, 127.f));
	RepCosmeticData.RightTrackEffectiveAngularSpeed = static_cast<int8>(FMath::Clamp(FMath::RoundHalfFromZero(RightTrackEffectiveAngularSpeed), -127.f, 127.f));
	RepCosmeticData.EffectiveSteeringAngularSpeed = static_cast<int8>(FMath::Clamp(FMath::RoundHalfFromZero(EffectiveSteeringAngularSpeed), -127.f, 127.f));
}

void UPrvVehicleMovementComponent::OnRep_RepCosmeticData()
{
	EngineRPM = static_cast<float>(RepCosmeticData.EngineRPM) / 255.f * MaxEngineRPM;
	LeftTrackEffectiveAngularSpeed = static_cast<float>(RepCosmeticData.LeftTrackEffectiveAngularSpeed);
	RightTrackEffectiveAngularSpeed = static_cast<float>(RepCosmeticData.RightTrackEffectiveAngularSpeed);
	EffectiveSteeringAngularSpeed = static_cast<float>(RepCosmeticData.EffectiveSteeringAngularSpeed);
}

void UPrvVehicleMovementComponent::GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const
{
	Super::GetLifetimeReplicatedProps(OutLifetimeProps);

	DOREPLIFETIME(UPrvVehicleMovementComponent, bIsSleeping);
	DOREPLIFETIME(UPrvVehicleMovementComponent, bIsMovementEnabled);

	if (bFakeAutonomousProxy)
	{
		DOREPLIFETIME(UPrvVehicleMovementComponent, RepCosmeticData);
	}
	else
	{
		DOREPLIFETIME_CONDITION(UPrvVehicleMovementComponent, RepCosmeticData, COND_SimulatedOnly);
	}
}
