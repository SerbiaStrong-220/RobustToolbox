/*
* Farseer Physics Engine:
* Copyright (c) 2012 Ian Qvist
*
* Original source Box2D:
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

using System;
using System.Numerics;
using System.Threading;
using System.Threading.Tasks;
using Robust.Shared.Maths;
using Robust.Shared.Physics.Collision;
using Robust.Shared.Physics.Components;
using Robust.Shared.Physics.Dynamics;
using Robust.Shared.Physics.Dynamics.Contacts;
using Robust.Shared.Utility;

namespace Robust.Shared.Physics.Systems;

public abstract partial class SharedPhysicsSystem
{
    private void ResetSolver(
        in SolverData data,
        in IslandData island,
        ContactVelocityConstraintSoA velocityConstraints,
        ContactPositionConstraintSoA positionConstraints)
    {
        var contactCount = island.Contacts.Count;

        // Build constraints
        // For now these are going to be bare but will change
        for (var i = 0; i < contactCount; i++)
        {
            var contact = island.Contacts[i];
            Fixture fixtureA = contact.FixtureA!;
            Fixture fixtureB = contact.FixtureB!;
            var shapeA = fixtureA.Shape;
            var shapeB = fixtureB.Shape;
            float radiusA = shapeA.Radius;
            float radiusB = shapeB.Radius;
            var bodyA = contact.BodyA!;
            var bodyB = contact.BodyB!;
            var manifold = contact.Manifold;

            int pointCount = manifold.PointCount;
            DebugTools.Assert(pointCount > 0);

            velocityConstraints.Friction[i] = contact.Friction;
            velocityConstraints.Restitution[i] = contact.Restitution;
            velocityConstraints.TangentSpeed[i] = contact.TangentSpeed;
            velocityConstraints.IndexA[i] = bodyA.IslandIndex[island.Index];
            velocityConstraints.IndexB[i] = bodyB.IslandIndex[island.Index];
            // Don't need to reset point data as it all gets set below.

            var (invMassA, invMassB) = GetInvMass(bodyA, bodyB);

            (velocityConstraints.InvMassA[i], velocityConstraints.InvMassB[i]) = (invMassA, invMassB);
            velocityConstraints.InvIA[i] = bodyA.InvI;
            velocityConstraints.InvIB[i] = bodyB.InvI;
            velocityConstraints.ContactIndex[i] = i;
            velocityConstraints.PointCount[i] = pointCount;

            velocityConstraints.K[i] = Vector4.Zero;
            velocityConstraints.NormalMass[i] = Vector4.Zero;

            positionConstraints.IndexA[i] = bodyA.IslandIndex[island.Index];
            positionConstraints.IndexB[i] = bodyB.IslandIndex[island.Index];
            (positionConstraints.InvMassA[i], positionConstraints.InvMassB[i]) = (invMassA, invMassB);
            positionConstraints.LocalCenterA[i] = bodyA.LocalCenter;
            positionConstraints.LocalCenterB[i] = bodyB.LocalCenter;

            positionConstraints.InvIA[i] = bodyA.InvI;
            positionConstraints.InvIB[i] = bodyB.InvI;
            positionConstraints.LocalNormal[i] = manifold.LocalNormal;
            positionConstraints.LocalPoint[i] = manifold.LocalPoint;
            positionConstraints.PointCount[i] = pointCount;
            positionConstraints.RadiusA[i] = radiusA;
            positionConstraints.RadiusB[i] = radiusB;
            positionConstraints.Type[i] = manifold.Type;
            var points = manifold.Points.AsSpan;
            var posPoints = positionConstraints.LocalPoints;

            for (var j = 0; j < pointCount; ++j)
            {
                var contactPoint = points[j];

                if (_warmStarting)
                {
                    velocityConstraints.PointNormalImpulse[i].AsSpan[j] = data.DtRatio * contactPoint.NormalImpulse;
                    velocityConstraints.PointTangentImpulse[i].AsSpan[j] = data.DtRatio * contactPoint.TangentImpulse;
                }
                else
                {
                    velocityConstraints.PointNormalImpulse[i].AsSpan[j] = 0.0f;
                    velocityConstraints.PointTangentImpulse[i].AsSpan[j] = 0.0f;
                }

                velocityConstraints.PointRelativeVelocityA[i].AsSpan[j] = Vector2.Zero;
                velocityConstraints.PointRelativeVelocityB[i].AsSpan[j] = Vector2.Zero;
                velocityConstraints.PointNormalMass[i].AsSpan[j] = 0.0f;
                velocityConstraints.PointTangentMass[i].AsSpan[j] = 0.0f;
                velocityConstraints.PointVelocityBias[i].AsSpan[j] = 0.0f;

                posPoints[i].AsSpan[j] = contactPoint.LocalPoint;
            }
        }
    }

    private (float, float) GetInvMass(PhysicsComponent bodyA, PhysicsComponent bodyB)
    {
        // God this is shitcodey but uhhhh we need to snowflake KinematicController for nice collisions.
        // TODO: Might need more finagling with the kinematic bodytype
        switch (bodyA.BodyType)
        {
            case BodyType.Kinematic:
            case BodyType.Static:
                return (bodyA.InvMass, bodyB.InvMass);
            case BodyType.KinematicController:
                switch (bodyB.BodyType)
                {
                    case BodyType.Kinematic:
                    case BodyType.Static:
                        return (bodyA.InvMass, bodyB.InvMass);
                    case BodyType.Dynamic:
                        return (bodyA.InvMass, 0f);
                    case BodyType.KinematicController:
                        return (0f, 0f);
                    default:
                        throw new ArgumentOutOfRangeException();
                }
            case BodyType.Dynamic:
                switch (bodyB.BodyType)
                {
                    case BodyType.Kinematic:
                    case BodyType.Static:
                    case BodyType.Dynamic:
                        return (bodyA.InvMass, bodyB.InvMass);
                    case BodyType.KinematicController:
                        return (0f, bodyB.InvMass);
                    default:
                        throw new ArgumentOutOfRangeException();
                }
            default:
                throw new ArgumentOutOfRangeException();
        }
    }

    private void InitializeVelocityConstraints(
        in SolverData data,
        in IslandData island,
        ContactVelocityConstraintSoA velocityConstraints,
        ContactPositionConstraintSoA positionConstraints,
        Vector2[] positions,
        float[] angles,
        Vector2[] linearVelocities,
        float[] angularVelocities)
    {
        Span<Vector2> points = stackalloc Vector2[2];
        var contactCount = island.Contacts.Count;
        var contacts = island.Contacts;
        var offset = island.Offset;

        for (var i = 0; i < contactCount; ++i)
        {
            var radiusA = positionConstraints.RadiusA[i];
            var radiusB = positionConstraints.RadiusB[i];
            var manifold = contacts[velocityConstraints.ContactIndex[i]].Manifold;

            var indexA = velocityConstraints.IndexA[i];
            var indexB = velocityConstraints.IndexB[i];

            var invMassA = velocityConstraints.InvMassA[i];
            var invMassB = velocityConstraints.InvMassB[i];
            var invIA = velocityConstraints.InvIA[i];
            var invIB = velocityConstraints.InvIB[i];
            var localCenterA = positionConstraints.LocalCenterA[i];
            var localCenterB = positionConstraints.LocalCenterB[i];

            var centerA = positions[indexA];
            var angleA = angles[indexA];
            var linVelocityA = linearVelocities[offset + indexA];
            var angVelocityA = angularVelocities[offset + indexA];

            var centerB = positions[indexB];
            var angleB = angles[indexB];
            var linVelocityB = linearVelocities[offset + indexB];
            var angVelocityB = angularVelocities[offset + indexB];

            DebugTools.Assert(manifold.PointCount > 0);

            var xfA = new Transform(angleA);
            var xfB = new Transform(angleB);
            xfA.Position = centerA - Physics.Transform.Mul(xfA.Quaternion2D, localCenterA);
            xfB.Position = centerB - Physics.Transform.Mul(xfB.Quaternion2D, localCenterB);

            InitializeManifold(ref manifold, xfA, xfB, radiusA, radiusB, out var normal, points);

            velocityConstraints.Normal[i] = normal;

            int pointCount = velocityConstraints.PointCount[i];

            for (int j = 0; j < pointCount; ++j)
            {
                velocityConstraints.PointRelativeVelocityA[i].AsSpan[j] = points[j] - centerA;
                velocityConstraints.PointRelativeVelocityB[i].AsSpan[j] = points[j] - centerB;

                float rnA = Vector2Helpers.Cross(velocityConstraints.PointRelativeVelocityA[i].AsSpan[j], velocityConstraints.Normal[i]);
                float rnB = Vector2Helpers.Cross(velocityConstraints.PointRelativeVelocityB[i].AsSpan[j], velocityConstraints.Normal[i]);

                float kNormal = invMassA + invMassB + invIA * rnA * rnA + invIB * rnB * rnB;

                velocityConstraints.PointNormalMass[i].AsSpan[j] = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

                Vector2 tangent = Vector2Helpers.Cross(velocityConstraints.Normal[i], 1.0f);

                float rtA = Vector2Helpers.Cross(velocityConstraints.PointRelativeVelocityA[i].AsSpan[j], tangent);
                float rtB = Vector2Helpers.Cross(velocityConstraints.PointRelativeVelocityB[i].AsSpan[j], tangent);

                float kTangent = invMassA + invMassB + invIA * rtA * rtA + invIB * rtB * rtB;

                velocityConstraints.PointTangentMass[i].AsSpan[j] = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

                // Setup a velocity bias for restitution.
                velocityConstraints.PointVelocityBias[i].AsSpan[j] = 0.0f;
                float vRel = Vector2.Dot(velocityConstraints.Normal[i], linVelocityB + Vector2Helpers.Cross(angVelocityB, velocityConstraints.PointRelativeVelocityB[i].AsSpan[j]) - linVelocityA - Vector2Helpers.Cross(angVelocityA, velocityConstraints.PointRelativeVelocityA[i].AsSpan[j]));
                if (vRel < -data.VelocityThreshold)
                {
                    velocityConstraints.PointVelocityBias[i].AsSpan[j] = -velocityConstraints.Restitution[i] * vRel;
                }
            }

            // If we have two points, then prepare the block solver.
            if (velocityConstraints.PointCount[i] == 2)
            {
                var rn1A = Vector2Helpers.Cross(velocityConstraints.PointRelativeVelocityA[i].AsSpan[0], velocityConstraints.Normal[i]);
                var rn1B = Vector2Helpers.Cross(velocityConstraints.PointRelativeVelocityB[i].AsSpan[0], velocityConstraints.Normal[i]);
                var rn2A = Vector2Helpers.Cross(velocityConstraints.PointRelativeVelocityA[i].AsSpan[1], velocityConstraints.Normal[i]);
                var rn2B = Vector2Helpers.Cross(velocityConstraints.PointRelativeVelocityB[i].AsSpan[1], velocityConstraints.Normal[i]);

                var k11 = invMassA + invMassB + invIA * rn1A * rn1A + invIB * rn1B * rn1B;
                var k22 = invMassA + invMassB + invIA * rn2A * rn2A + invIB * rn2B * rn2B;
                var k12 = invMassA + invMassB + invIA * rn1A * rn2A + invIB * rn1B * rn2B;

                // Ensure a reasonable condition number.
                const float k_maxConditionNumber = 1000.0f;
                if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12))
                {
                    // K is safe to invert.
                    velocityConstraints.K[i] = new Vector4(k11, k12, k12, k22);

                    velocityConstraints.NormalMass[i] = Vector4Helpers.Inverse(velocityConstraints.K[i]);
                }
                else
                {
                    // The constraints are redundant, just use one.
                    // TODO_ERIN use deepest?
                    velocityConstraints.PointCount[i] = 1;
                }
            }
        }
    }

    private void WarmStart(
        in SolverData data,
        in IslandData island,
        ContactVelocityConstraintSoA velocityConstraints,
        Vector2[] linearVelocities,
        float[] angularVelocities)
    {
        var offset = island.Offset;

        for (var i = 0; i < island.Contacts.Count; ++i)
        {
            var indexA = velocityConstraints.IndexA[i];
            var indexB = velocityConstraints.IndexB[i];
            var invMassA = velocityConstraints.InvMassA[i];
            var invIA = velocityConstraints.InvIA[i];
            var invMassB = velocityConstraints.InvMassB[i];
            var invIB = velocityConstraints.InvIB[i];
            var pointCount = velocityConstraints.PointCount[i];

            ref var linVelocityA = ref linearVelocities[offset + indexA];
            ref var angVelocityA = ref angularVelocities[offset + indexA];
            ref var linVelocityB = ref linearVelocities[offset + indexB];
            ref var angVelocityB = ref angularVelocities[offset + indexB];

            var normal = velocityConstraints.Normal[i];
            // var tangent = Vector2Helpers.Cross(normal, 1.0f);
            var tangent = new Vector2(velocityConstraints.TangentX[i], velocityConstraints.TangentY[i]);

            for (var j = 0; j < pointCount; ++j)
            {
                var P = normal * velocityConstraints.PointNormalImpulse[i].AsSpan[j] + tangent * velocityConstraints.PointTangentImpulse[i].AsSpan[j];
                angVelocityA -= invIA * Vector2Helpers.Cross(velocityConstraints.PointRelativeVelocityA[i].AsSpan[j], P);
                linVelocityA -= P * invMassA;
                angVelocityB += invIB * Vector2Helpers.Cross(velocityConstraints.PointRelativeVelocityB[i].AsSpan[j], P);
                linVelocityB += P * invMassB;
            }
        }
    }

    private static void SolveVelocityConstraints(
        in IslandData island,
        ParallelOptions? options,
        ContactVelocityConstraintSoA velocityConstraints,
        Vector2[] linearVelocities,
        float[] angularVelocities)
    {
        var contactCount = island.Contacts.Count;

        if (options != null && contactCount > VelocityConstraintsPerThread * 2)
        {
            static void ProcessParallelInternal(
                IslandData island,
                int contactCount,
                ParallelOptions options,
                ContactVelocityConstraintSoA velocityConstraints,
                Vector2[] linearVelocities,
                float[] angularVelocities)
            {
                var batches = (int) Math.Ceiling((float) contactCount / VelocityConstraintsPerThread);

                Parallel.For(0, batches, options, i =>
                {
                    var start = i * VelocityConstraintsPerThread;
                    var end = Math.Min(start + VelocityConstraintsPerThread, contactCount);
                    SolveVelocityConstraints(island, start, end, velocityConstraints, linearVelocities, angularVelocities);
                });
            }

            ProcessParallelInternal(
                island,
                contactCount,
                options,
                velocityConstraints,
                linearVelocities,
                angularVelocities);
        }
        else
        {
            SolveVelocityConstraints(in island, 0, contactCount, velocityConstraints, linearVelocities, angularVelocities);
        }
    }

    private static void SolveVelocityConstraints(
        in IslandData island,
        int start,
        int end,
        ContactVelocityConstraintSoA velocityConstraints,
        Vector2[] linearVelocities,
        float[] angularVelocities)
    {
        var offset = island.Offset;

        // Here be dragons
        for (var i = start; i < end; ++i)
        {
            var indexA = velocityConstraints.IndexA[i];
            var indexB = velocityConstraints.IndexB[i];
            var mA = velocityConstraints.InvMassA[i];
            var iA = velocityConstraints.InvIA[i];
            var mB = velocityConstraints.InvMassB[i];
            var iB = velocityConstraints.InvIB[i];
            var pointCount = velocityConstraints.PointCount[i];

            ref var vA = ref linearVelocities[offset + indexA];
            ref var wA = ref angularVelocities[offset + indexA];
            ref var vB = ref linearVelocities[offset + indexB];
            ref var wB = ref angularVelocities[offset + indexB];

            var normal = velocityConstraints.Normal[i];
            var tangent = Vector2Helpers.Cross(normal, 1.0f);
            var friction = velocityConstraints.Friction[i];

            DebugTools.Assert(pointCount is 1 or 2);

            // Solve tangent constraints first because non-penetration is more important
            // than friction.
            for (var j = 0; j < pointCount; ++j)
            {
                // Relative velocity at contact
                var dv = vB + Vector2Helpers.Cross(wB, velocityConstraints.PointRelativeVelocityB[i].AsSpan[j]) - vA - Vector2Helpers.Cross(wA, velocityConstraints.PointRelativeVelocityA[i].AsSpan[j]);

                // Compute tangent force
                float vt = Vector2.Dot(dv, tangent) - velocityConstraints.TangentSpeed[i];
                float lambda = velocityConstraints.PointTangentMass[i].AsSpan[j] * (-vt);

                // b2Clamp the accumulated force
                var maxFriction = friction * velocityConstraints.PointNormalImpulse[i].AsSpan[j];
                var newImpulse = Math.Clamp(velocityConstraints.PointTangentImpulse[i].AsSpan[j] + lambda, -maxFriction, maxFriction);
                lambda = newImpulse - velocityConstraints.PointTangentImpulse[i].AsSpan[j];
                velocityConstraints.PointTangentImpulse[i].AsSpan[j] = newImpulse;

                // Apply contact impulse
                Vector2 P = tangent * lambda;

                vA -= P * mA;
                wA -= iA * Vector2Helpers.Cross(velocityConstraints.PointRelativeVelocityA[i].AsSpan[j], P);

                vB += P * mB;
                wB += iB * Vector2Helpers.Cross(velocityConstraints.PointRelativeVelocityB[i].AsSpan[j], P);
            }

            // Solve normal constraints
            if (velocityConstraints.PointCount[i] == 1)
            {
                // Relative velocity at contact
                Vector2 dv = vB + Vector2Helpers.Cross(wB, velocityConstraints.PointRelativeVelocityB[i].AsSpan[0]) - vA - Vector2Helpers.Cross(wA, velocityConstraints.PointRelativeVelocityA[i].AsSpan[0]);

                // Compute normal impulse
                float vn = Vector2.Dot(dv, normal);
                float lambda = -velocityConstraints.PointNormalMass[i].AsSpan[0] * (vn - velocityConstraints.PointVelocityBias[i].AsSpan[0]);

                // b2Clamp the accumulated impulse
                float newImpulse = Math.Max(velocityConstraints.PointNormalImpulse[i].AsSpan[0] + lambda, 0.0f);
                lambda = newImpulse - velocityConstraints.PointNormalImpulse[i].AsSpan[0];
                velocityConstraints.PointNormalImpulse[i].AsSpan[0] = newImpulse;

                // Apply contact impulse
                Vector2 P = normal * lambda;
                vA -= P * mA;
                wA -= iA * Vector2Helpers.Cross(velocityConstraints.PointRelativeVelocityA[i].AsSpan[0], P);

                vB += P * mB;
                wB += iB * Vector2Helpers.Cross(velocityConstraints.PointRelativeVelocityB[i].AsSpan[0], P);
            }
            else
            {
                // Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
                // Build the mini LCP for this contact patch
                //
                // vn = A * x + b, vn >= 0, , vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
                //
                // A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
                // b = vn0 - velocityBias
                //
                // The system is solved using the "Total enumeration method" (s. Murty). The complementary constraint vn_i * x_i
                // implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D contact problem the cases
                // vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be tested. The first valid
                // solution that satisfies the problem is chosen.
                //
                // In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which only requires
                // that the accumulated impulse is clamped and not the incremental impulse) we change the impulse variable (x_i).
                //
                // Substitute:
                //
                // x = a + d
                //
                // a := old total impulse
                // x := new total impulse
                // d := incremental impulse
                //
                // For the current iteration we extend the formula for the incremental impulse
                // to compute the new total impulse:
                //
                // vn = A * d + b
                //    = A * (x - a) + b
                //    = A * x + b - A * a
                //    = A * x + b'
                // b' = b - A * a;

                Vector2 a = new Vector2(velocityConstraints.PointNormalImpulse[i].AsSpan[0], velocityConstraints.PointNormalImpulse[i].AsSpan[1]);
                DebugTools.Assert(a.X >= 0.0f && a.Y >= 0.0f);

                // Relative velocity at contact
                Vector2 dv1 = vB + Vector2Helpers.Cross(wB, velocityConstraints.PointRelativeVelocityB[i].AsSpan[0]) - vA - Vector2Helpers.Cross(wA, velocityConstraints.PointRelativeVelocityA[i].AsSpan[0]);
                Vector2 dv2 = vB + Vector2Helpers.Cross(wB, velocityConstraints.PointRelativeVelocityB[i].AsSpan[1]) - vA - Vector2Helpers.Cross(wA, velocityConstraints.PointRelativeVelocityA[i].AsSpan[1]);

                // Compute normal velocity
                float vn1 = Vector2.Dot(dv1, normal);
                float vn2 = Vector2.Dot(dv2, normal);

                Vector2 b = new Vector2
                {
                    X = vn1 - velocityConstraints.PointVelocityBias[i].AsSpan[0],
                    Y = vn2 - velocityConstraints.PointVelocityBias[i].AsSpan[1]
                };

                // Compute b'
                b -= Physics.Transform.Mul(velocityConstraints.K[i], a);

                //const float k_errorTol = 1e-3f;
                //B2_NOT_USED(k_errorTol);

                for (; ; )
                {
                    //
                    // Case 1: vn = 0
                    //
                    // 0 = A * x + b'
                    //
                    // Solve for x:
                    //
                    // x = - inv(A) * b'
                    //
                    Vector2 x = -Physics.Transform.Mul(velocityConstraints.NormalMass[i], b);

                    if (x.X >= 0.0f && x.Y >= 0.0f)
                    {
                        // Get the incremental impulse
                        Vector2 d = x - a;

                        // Apply incremental impulse
                        Vector2 P1 = normal * d.X;
                        Vector2 P2 = normal * d.Y;
                        vA -= (P1 + P2) * mA;
                        wA -= iA * (Vector2Helpers.Cross(velocityConstraints.PointRelativeVelocityA[i].AsSpan[0], P1) + Vector2Helpers.Cross(velocityConstraints.PointRelativeVelocityA[i].AsSpan[1], P2));

                        vB += (P1 + P2) * mB;
                        wB += iB * (Vector2Helpers.Cross(velocityConstraints.PointRelativeVelocityB[i].AsSpan[0], P1) + Vector2Helpers.Cross(velocityConstraints.PointRelativeVelocityB[i].AsSpan[1], P2));

                        // Accumulate
                        velocityConstraints.PointNormalImpulse[i].AsSpan[0] = x.X;
                        velocityConstraints.PointNormalImpulse[i].AsSpan[1] = x.Y;

                        break;
                    }

                    //
                    // Case 2: vn1 = 0 and x2 = 0
                    //
                    //   0 = a11 * x1 + a12 * 0 + b1'
                    // vn2 = a21 * x1 + a22 * 0 + b2'
                    //
                    x.X = -velocityConstraints.PointNormalMass[i].AsSpan[0] * b.X;
                    x.Y = 0.0f;
                    vn1 = 0.0f;
                    vn2 = velocityConstraints.K[i].Y * x.X + b.Y;

                    if (x.X >= 0.0f && vn2 >= 0.0f)
                    {
                        // Get the incremental impulse
                        Vector2 d = x - a;

                        // Apply incremental impulse
                        Vector2 P1 = normal * d.X;
                        Vector2 P2 = normal * d.Y;
                        vA -= (P1 + P2) * mA;
                        wA -= iA * (Vector2Helpers.Cross(velocityConstraints.PointRelativeVelocityA[i].AsSpan[0], P1) + Vector2Helpers.Cross(velocityConstraints.PointRelativeVelocityA[i].AsSpan[1], P2));

                        vB += (P1 + P2) * mB;
                        wB += iB * (Vector2Helpers.Cross(velocityConstraints.PointRelativeVelocityB[i].AsSpan[0], P1) + Vector2Helpers.Cross(velocityConstraints.PointRelativeVelocityB[i].AsSpan[1], P2));

                        // Accumulate
                        velocityConstraints.PointNormalImpulse[i].AsSpan[0] = x.X;
                        velocityConstraints.PointNormalImpulse[i].AsSpan[1] = x.Y;

                        break;
                    }


                    //
                    // Case 3: vn2 = 0 and x1 = 0
                    //
                    // vn1 = a11 * 0 + a12 * x2 + b1'
                    //   0 = a21 * 0 + a22 * x2 + b2'
                    //
                    x.X = 0.0f;
                    x.Y = -velocityConstraints.PointNormalMass[i].AsSpan[1] * b.Y;
                    vn1 = velocityConstraints.K[i].Z * x.Y + b.X;
                    vn2 = 0.0f;

                    if (x.Y >= 0.0f && vn1 >= 0.0f)
                    {
                        // Resubstitute for the incremental impulse
                        Vector2 d = x - a;

                        // Apply incremental impulse
                        Vector2 P1 = normal * d.X;
                        Vector2 P2 = normal * d.Y;
                        vA -= (P1 + P2) * mA;
                        wA -= iA * (Vector2Helpers.Cross(velocityConstraints.PointRelativeVelocityA[i].AsSpan[0], P1) + Vector2Helpers.Cross(velocityConstraints.PointRelativeVelocityA[i].AsSpan[1], P2));

                        vB += (P1 + P2) * mB;
                        wB += iB * (Vector2Helpers.Cross(velocityConstraints.PointRelativeVelocityB[i].AsSpan[0], P1) + Vector2Helpers.Cross(velocityConstraints.PointRelativeVelocityB[i].AsSpan[1], P2));

                        // Accumulate
                        velocityConstraints.PointNormalImpulse[i].AsSpan[0] = x.X;
                        velocityConstraints.PointNormalImpulse[i].AsSpan[1] = x.Y;

                        break;
                    }

                    //
                    // Case 4: x1 = 0 and x2 = 0
                    //
                    // vn1 = b1
                    // vn2 = b2;
                    x.X = 0.0f;
                    x.Y = 0.0f;
                    vn1 = b.X;
                    vn2 = b.Y;

                    if (vn1 >= 0.0f && vn2 >= 0.0f)
                    {
                        // Resubstitute for the incremental impulse
                        Vector2 d = x - a;

                        // Apply incremental impulse
                        Vector2 P1 = normal * d.X;
                        Vector2 P2 = normal * d.Y;
                        vA -= (P1 + P2) * mA;
                        wA -= iA * (Vector2Helpers.Cross(velocityConstraints.PointRelativeVelocityA[i].AsSpan[0], P1) + Vector2Helpers.Cross(velocityConstraints.PointRelativeVelocityA[i].AsSpan[1], P2));

                        vB += (P1 + P2) * mB;
                        wB += iB * (Vector2Helpers.Cross(velocityConstraints.PointRelativeVelocityB[i].AsSpan[0], P1) + Vector2Helpers.Cross(velocityConstraints.PointRelativeVelocityB[i].AsSpan[1], P2));

                        // Accumulate
                        velocityConstraints.PointNormalImpulse[i].AsSpan[0] = x.X;
                        velocityConstraints.PointNormalImpulse[i].AsSpan[1] = x.Y;

                        break;
                    }

                    // No solution, give up. This is hit sometimes, but it doesn't seem to matter.
                    break;
                }
            }
        }
    }

    private void StoreImpulses(in IslandData island, ContactVelocityConstraintSoA velocityConstraints)
    {
        for (var i = 0; i < island.Contacts.Count; ++i)
        {
            ref var manifold = ref island.Contacts[velocityConstraints.ContactIndex[i]].Manifold;
            var manPoints = manifold.Points.AsSpan;

            for (var j = 0; j < velocityConstraints.PointCount[i]; ++j)
            {
                ref var point = ref manPoints[j];
                point.NormalImpulse = velocityConstraints.PointNormalImpulse[i].AsSpan[j];
                point.TangentImpulse = velocityConstraints.PointTangentImpulse[i].AsSpan[j];
            }
        }
    }

    private static bool SolvePositionConstraints(
        in SolverData data,
        in IslandData island,
        ParallelOptions? options,
        ContactPositionConstraintSoA positionConstraints,
        Vector2[] positions,
        float[] angles)
    {
        var contactCount = island.Contacts.Count;

        // Parallel
        if (options != null && contactCount > PositionConstraintsPerThread * 2)
        {
            static bool ProcessParallelInternal(
                int contactCount,
                SolverData data,
                ParallelOptions options,
                ContactPositionConstraintSoA positionConstraints,
                Vector2[] positions,
                float[] angles)
            {
                var unsolved = 0;
                var batches = (int) Math.Ceiling((float) contactCount / PositionConstraintsPerThread);

                Parallel.For(0, batches, options, i =>
                {
                    var start = i * PositionConstraintsPerThread;
                    var end = Math.Min(start + PositionConstraintsPerThread, contactCount);

                    if (!SolvePositionConstraints(data, start, end, positionConstraints, positions, angles))
                        Interlocked.Increment(ref unsolved);
                });

                return unsolved == 0;
            }

            return ProcessParallelInternal(contactCount, data, options, positionConstraints, positions, angles);
        }

        // No parallel
        return SolvePositionConstraints(data, 0, contactCount, positionConstraints, positions, angles);
    }

    /// <summary>
    ///     Tries to solve positions for all contacts specified.
    /// </summary>
    /// <returns>true if all positions solved</returns>
    private static bool SolvePositionConstraints(
        in SolverData data,
        int start,
        int end,
        ContactPositionConstraintSoA positionConstraints,
        Vector2[] positions,
        float[] angles)
    {
        float minSeparation = 0.0f;

        for (int i = start; i < end; ++i)
        {
            int indexA = positionConstraints.IndexA[i];
            int indexB = positionConstraints.IndexB[i];
            Vector2 localCenterA = positionConstraints.LocalCenterA[i];
            float mA = positionConstraints.InvMassA[i];
            float iA = positionConstraints.InvIA[i];
            Vector2 localCenterB = positionConstraints.LocalCenterB[i];
            float mB = positionConstraints.InvMassB[i];
            float iB = positionConstraints.InvIB[i];
            int pointCount = positionConstraints.PointCount[i];

            ref var centerA = ref positions[indexA];
            ref var angleA = ref angles[indexA];
            ref var centerB = ref positions[indexB];
            ref var angleB = ref angles[indexB];

            // Solve normal constraints
            for (int j = 0; j < pointCount; ++j)
            {
                Transform xfA = new Transform(angleA);
                Transform xfB = new Transform(angleB);
                xfA.Position = centerA - Physics.Transform.Mul(xfA.Quaternion2D, localCenterA);
                xfB.Position = centerB - Physics.Transform.Mul(xfB.Quaternion2D, localCenterB);

                PositionSolverManifoldInitialize(positionConstraints, i, j, xfA, xfB, out var normal, out var point, out var separation);

                Vector2 rA = point - centerA;
                Vector2 rB = point - centerB;

                // Track max constraint error.
                minSeparation = Math.Min(minSeparation, separation);

                // Prevent large corrections and allow slop.
                float C = Math.Clamp(data.Baumgarte * (separation + PhysicsConstants.LinearSlop), -data.MaxLinearCorrection, 0.0f);

                // Compute the effective mass.
                float rnA = Vector2Helpers.Cross(rA, normal);
                float rnB = Vector2Helpers.Cross(rB, normal);
                float K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

                // Compute normal impulse
                float impulse = K > 0.0f ? -C / K : 0.0f;

                Vector2 P = normal * impulse;

                centerA -= P * mA;
                angleA -= iA * Vector2Helpers.Cross(rA, P);

                centerB += P * mB;
                angleB += iB * Vector2Helpers.Cross(rB, P);
            }
        }

        // We can't expect minSpeparation >= -b2_linearSlop because we don't
        // push the separation above -b2_linearSlop.
        return minSeparation >= -3.0f * PhysicsConstants.LinearSlop;
    }

    /// <summary>
    /// Evaluate the manifold with supplied transforms. This assumes
    /// modest motion from the original state. This does not change the
    /// point count, impulses, etc. The radii must come from the Shapes
    /// that generated the manifold.
    /// </summary>
    internal static void InitializeManifold(
        ref Manifold manifold,
        in Transform xfA,
        in Transform xfB,
        float radiusA,
        float radiusB,
        out Vector2 normal,
        Span<Vector2> points)
    {
        normal = Vector2.Zero;

        if (manifold.PointCount == 0)
        {
            return;
        }

        switch (manifold.Type)
        {
            case ManifoldType.Circles:
            {
                normal = new Vector2(1.0f, 0.0f);
                Vector2 pointA = Physics.Transform.Mul(xfA, manifold.LocalPoint);
                Vector2 pointB = Physics.Transform.Mul(xfB, manifold.Points._00.LocalPoint);

                if ((pointA - pointB).LengthSquared() > float.Epsilon * float.Epsilon)
                {
                    normal = pointB - pointA;
                    normal = normal.Normalized();
                }

                Vector2 cA = pointA + normal * radiusA;
                Vector2 cB = pointB - normal * radiusB;
                points[0] = (cA + cB) * 0.5f;
            }
            break;

            case ManifoldType.FaceA:
            {
                normal = Physics.Transform.Mul(xfA.Quaternion2D, manifold.LocalNormal);
                Vector2 planePoint = Physics.Transform.Mul(xfA, manifold.LocalPoint);
                var manPoints = manifold.Points.AsSpan;

                for (int i = 0; i < manifold.PointCount; ++i)
                {
                    Vector2 clipPoint = Physics.Transform.Mul(xfB, manPoints[i].LocalPoint);
                    Vector2 cA = clipPoint + normal * (radiusA - Vector2.Dot(clipPoint - planePoint, normal));
                    Vector2 cB = clipPoint - normal * radiusB;
                    points[i] = (cA + cB) * 0.5f;
                }
            }
            break;

            case ManifoldType.FaceB:
            {
                normal = Physics.Transform.Mul(xfB.Quaternion2D, manifold.LocalNormal);
                Vector2 planePoint = Physics.Transform.Mul(xfB, manifold.LocalPoint);
                var manPoints = manifold.Points.AsSpan;

                for (int i = 0; i < manifold.PointCount; ++i)
                {
                    Vector2 clipPoint = Physics.Transform.Mul(xfA, manPoints[i].LocalPoint);
                    Vector2 cB = clipPoint + normal * (radiusB - Vector2.Dot(clipPoint - planePoint, normal));
                    Vector2 cA = clipPoint - normal * radiusA;
                    points[i] = (cA + cB) * 0.5f;
                }

                // Ensure normal points from A to B.
                normal = -normal;
            }
            break;
            default:
                // Shouldn't happentm
                throw new InvalidOperationException();

        }
    }

    private static void PositionSolverManifoldInitialize(
        in ContactPositionConstraintSoA pc,
        int pcIndex,
        int index,
        in Transform xfA,
        in Transform xfB,
        out Vector2 normal,
        out Vector2 point,
        out float separation)
    {
        DebugTools.Assert(pc.PointCount[pcIndex] > 0);

            switch (pc.Type[pcIndex])
            {
                case ManifoldType.Circles:
                    {
                        Vector2 pointA = Physics.Transform.Mul(xfA, pc.LocalPoint[pcIndex]);
                        Vector2 pointB = Physics.Transform.Mul(xfB, pc.LocalPoints[pcIndex]._00);
                        normal = pointB - pointA;

                        //FPE: Fix to handle zero normalization
                        if (normal != Vector2.Zero)
                            normal = normal.Normalized();

                        point = (pointA + pointB) * 0.5f;
                        separation = Vector2.Dot(pointB - pointA, normal) - pc.RadiusA[pcIndex] - pc.RadiusB[pcIndex];
                    }
                    break;

                case ManifoldType.FaceA:
                    {
                        var pcPoints = pc.LocalPoints[pcIndex].AsSpan;
                        normal = Physics.Transform.Mul(xfA.Quaternion2D, pc.LocalNormal[pcIndex]);
                        Vector2 planePoint = Physics.Transform.Mul(xfA, pc.LocalPoint[pcIndex]);

                        Vector2 clipPoint = Physics.Transform.Mul(xfB, pcPoints[index]);
                        separation = Vector2.Dot(clipPoint - planePoint, normal) - pc.RadiusA[pcIndex] - pc.RadiusB[pcIndex];
                        point = clipPoint;
                    }
                    break;

                case ManifoldType.FaceB:
                    {
                        var pcPoints = pc.LocalPoints[pcIndex].AsSpan;
                        normal = Physics.Transform.Mul(xfB.Quaternion2D, pc.LocalNormal[pcIndex]);
                        Vector2 planePoint = Physics.Transform.Mul(xfB, pc.LocalPoint[pcIndex]);

                        Vector2 clipPoint = Physics.Transform.Mul(xfA, pcPoints[index]);
                        separation = Vector2.Dot(clipPoint - planePoint, normal) - pc.RadiusA[pcIndex] - pc.RadiusB[pcIndex];
                        point = clipPoint;

                        // Ensure normal points from A to B
                        normal = -normal;
                    }
                    break;
                default:
                    normal = Vector2.Zero;
                    point = Vector2.Zero;
                    separation = 0;
                    break;

            }
    }
}
