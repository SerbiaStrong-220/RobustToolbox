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
// SS220: This file isn't original but is based on the solver code in ContactSolver.cs and optimized for SIMD processing

using System;
using System.Numerics;
using System.Threading;
using System.Threading.Tasks;
using Robust.Shared.Maths;
using Robust.Shared.Physics.Dynamics;
using Robust.Shared.Physics.Dynamics.Contacts;
using Robust.Shared.Utility;

namespace Robust.Shared.Physics.Systems;

public abstract partial class SharedPhysicsSystem
{
    private static void SolveMultiPointVelocityConstraints(
        in IslandData island,
        int[] contactIndices,
        int contactCount,
        ContactVelocityConstraintSoA vc,
        Vector2[] linearVelocities,
        float[] angularVelocities)
    {
        var offset = island.Offset;

        for (int idx = 0; idx < contactCount; idx++)
        {
            var i = contactIndices[idx];
            var indexA = vc.IndexA[i];
            var indexB = vc.IndexB[i];
            var mA = vc.InvMassA[i];
            var iA = vc.InvIA[i];
            var mB = vc.InvMassB[i];
            var iB = vc.InvIB[i];

            ref var vA = ref linearVelocities[offset + indexA];
            ref var wA = ref angularVelocities[offset + indexA];
            ref var vB = ref linearVelocities[offset + indexB];
            ref var wB = ref angularVelocities[offset + indexB];

            var normal = vc.Normal[i];
            var tangent = Vector2Helpers.Cross(normal, 1.0f);
            var friction = vc.Friction[i];

            for (var j = 0; j < 2; ++j)
            {
                var dv = vB + Vector2Helpers.Cross(wB, vc.PointRelativeVelocityB[i].AsSpan[j])
                    - vA - Vector2Helpers.Cross(wA, vc.PointRelativeVelocityA[i].AsSpan[j]);

                float vt = Vector2.Dot(dv, tangent) - vc.TangentSpeed[i];
                float lambda = vc.PointTangentMass[i].AsSpan[j] * (-vt);

                var maxFriction = friction * vc.PointNormalImpulse[i].AsSpan[j];
                var newImpulse = Math.Clamp(vc.PointTangentImpulse[i].AsSpan[j] + lambda, -maxFriction, maxFriction);
                lambda = newImpulse - vc.PointTangentImpulse[i].AsSpan[j];
                vc.PointTangentImpulse[i].AsSpan[j] = newImpulse;

                Vector2 P = tangent * lambda;

                vA -= P * mA;
                wA -= iA * Vector2Helpers.Cross(vc.PointRelativeVelocityA[i].AsSpan[j], P);

                vB += P * mB;
                wB += iB * Vector2Helpers.Cross(vc.PointRelativeVelocityB[i].AsSpan[j], P);
            }

            Vector2 a = new Vector2(vc.PointNormalImpulse[i].AsSpan[0],
                                    vc.PointNormalImpulse[i].AsSpan[1]);
            DebugTools.Assert(a.X >= 0.0f && a.Y >= 0.0f);

            // Relative velocities at both contact points
            Vector2 dv1 = vB + Vector2Helpers.Cross(wB, vc.PointRelativeVelocityB[i].AsSpan[0])
                        - vA - Vector2Helpers.Cross(wA, vc.PointRelativeVelocityA[i].AsSpan[0]);
            Vector2 dv2 = vB + Vector2Helpers.Cross(wB, vc.PointRelativeVelocityB[i].AsSpan[1])
                        - vA - Vector2Helpers.Cross(wA, vc.PointRelativeVelocityA[i].AsSpan[1]);

            float vn1 = Vector2.Dot(dv1, normal);
            float vn2 = Vector2.Dot(dv2, normal);

            Vector2 b = new Vector2
            {
                X = vn1 - vc.PointVelocityBias[i].AsSpan[0],
                Y = vn2 - vc.PointVelocityBias[i].AsSpan[1]
            };

            b -= Physics.Transform.Mul(vc.K[i], a);

            for (;;)
            {
                // Case 1: vn = 0
                Vector2 x = -Physics.Transform.Mul(vc.NormalMass[i], b);

                if (x.X >= 0.0f && x.Y >= 0.0f)
                {
                    Vector2 d = x - a;
                    Vector2 P1 = normal * d.X;
                    Vector2 P2 = normal * d.Y;
                    vA -= (P1 + P2) * mA;
                    wA -= iA * (Vector2Helpers.Cross(vc.PointRelativeVelocityA[i].AsSpan[0], P1)
                            + Vector2Helpers.Cross(vc.PointRelativeVelocityA[i].AsSpan[1], P2));
                    vB += (P1 + P2) * mB;
                    wB += iB * (Vector2Helpers.Cross(vc.PointRelativeVelocityB[i].AsSpan[0], P1)
                            + Vector2Helpers.Cross(vc.PointRelativeVelocityB[i].AsSpan[1], P2));
                    vc.PointNormalImpulse[i].AsSpan[0] = x.X;
                    vc.PointNormalImpulse[i].AsSpan[1] = x.Y;
                    break;
                }

                // Case 2: vn1 = 0 and x2 = 0
                x.X = -vc.PointNormalMass[i].AsSpan[0] * b.X;
                x.Y = 0.0f;
                vn1 = 0.0f;
                vn2 = vc.K[i].Y * x.X + b.Y;

                if (x.X >= 0.0f && vn2 >= 0.0f)
                {
                    Vector2 d = x - a;
                    Vector2 P1 = normal * d.X;
                    Vector2 P2 = normal * d.Y;
                    vA -= (P1 + P2) * mA;
                    wA -= iA * (Vector2Helpers.Cross(vc.PointRelativeVelocityA[i].AsSpan[0], P1)
                            + Vector2Helpers.Cross(vc.PointRelativeVelocityA[i].AsSpan[1], P2));
                    vB += (P1 + P2) * mB;
                    wB += iB * (Vector2Helpers.Cross(vc.PointRelativeVelocityB[i].AsSpan[0], P1)
                            + Vector2Helpers.Cross(vc.PointRelativeVelocityB[i].AsSpan[1], P2));
                    vc.PointNormalImpulse[i].AsSpan[0] = x.X;
                    vc.PointNormalImpulse[i].AsSpan[1] = x.Y;
                    break;
                }

                // Case 3: vn2 = 0 and x1 = 0
                x.X = 0.0f;
                x.Y = -vc.PointNormalMass[i].AsSpan[1] * b.Y;
                vn1 = vc.K[i].Z * x.Y + b.X;
                vn2 = 0.0f;

                if (x.Y >= 0.0f && vn1 >= 0.0f)
                {
                    Vector2 d = x - a;
                    Vector2 P1 = normal * d.X;
                    Vector2 P2 = normal * d.Y;
                    vA -= (P1 + P2) * mA;
                    wA -= iA * (Vector2Helpers.Cross(vc.PointRelativeVelocityA[i].AsSpan[0], P1)
                            + Vector2Helpers.Cross(vc.PointRelativeVelocityA[i].AsSpan[1], P2));
                    vB += (P1 + P2) * mB;
                    wB += iB * (Vector2Helpers.Cross(vc.PointRelativeVelocityB[i].AsSpan[0], P1)
                            + Vector2Helpers.Cross(vc.PointRelativeVelocityB[i].AsSpan[1], P2));
                    vc.PointNormalImpulse[i].AsSpan[0] = x.X;
                    vc.PointNormalImpulse[i].AsSpan[1] = x.Y;
                    break;
                }

                // Case 4: x1 = 0 and x2 = 0
                x.X = 0.0f;
                x.Y = 0.0f;
                vn1 = b.X;
                vn2 = b.Y;

                if (vn1 >= 0.0f && vn2 >= 0.0f)
                {
                    Vector2 d = x - a;
                    Vector2 P1 = normal * d.X;
                    Vector2 P2 = normal * d.Y;
                    vA -= (P1 + P2) * mA;
                    wA -= iA * (Vector2Helpers.Cross(vc.PointRelativeVelocityA[i].AsSpan[0], P1)
                            + Vector2Helpers.Cross(vc.PointRelativeVelocityA[i].AsSpan[1], P2));
                    vB += (P1 + P2) * mB;
                    wB += iB * (Vector2Helpers.Cross(vc.PointRelativeVelocityB[i].AsSpan[0], P1)
                            + Vector2Helpers.Cross(vc.PointRelativeVelocityB[i].AsSpan[1], P2));
                    vc.PointNormalImpulse[i].AsSpan[0] = x.X;
                    vc.PointNormalImpulse[i].AsSpan[1] = x.Y;
                    break;
                }

                // No solution
                break;
            }
        }
    }

    private static void SolveSinglePointVelocityConstraintsSIMD(
        in IslandData island,
        int[] contactIndices,
        int contactCount,
        ContactVelocityConstraintSoA vc,
        Vector2[] linearVelocities,
        float[] angularVelocities)
    {
        if (contactCount == 0) return;

        int bodyCount = island.Bodies.Count;
        int offset = island.Offset;

        // Temporary delta arrays – one set for the entire solve pass
        Span<float> deltaLinAX = stackalloc float[bodyCount];
        Span<float> deltaLinAY = stackalloc float[bodyCount];
        Span<float> deltaAngA  = stackalloc float[bodyCount];
        Span<float> deltaLinBX = stackalloc float[bodyCount];
        Span<float> deltaLinBY = stackalloc float[bodyCount];
        Span<float> deltaAngB  = stackalloc float[bodyCount];

        deltaLinAX.Clear();
        deltaLinAY.Clear();
        deltaAngA.Clear();
        deltaLinBX.Clear();
        deltaLinBY.Clear();
        deltaAngB.Clear();

        int vectorSize = Vector<float>.Count;
        int batchStart = 0;

        {
            // Load per‑contact constants into vector-sized arrays
            Span<float> mA   = stackalloc float[vectorSize];
            Span<float> iA   = stackalloc float[vectorSize];
            Span<float> mB   = stackalloc float[vectorSize];
            Span<float> iB   = stackalloc float[vectorSize];
            Span<float> nX   = stackalloc float[vectorSize];
            Span<float> nY   = stackalloc float[vectorSize];
            Span<float> tX   = stackalloc float[vectorSize];
            Span<float> tY   = stackalloc float[vectorSize];
            Span<float> tM   = stackalloc float[vectorSize]; // tangent mass
            Span<float> nM   = stackalloc float[vectorSize]; // normal mass
            Span<float> bias = stackalloc float[vectorSize];
            Span<float> friction = stackalloc float[vectorSize];
            Span<float> tangSpeed = stackalloc float[vectorSize];
            Span<float> accN  = stackalloc float[vectorSize]; // accumulated normal impulse
            Span<float> accT  = stackalloc float[vectorSize]; // accumulated tangent impulse
            Span<float> rAX  = stackalloc float[vectorSize];
            Span<float> rAY  = stackalloc float[vectorSize];
            Span<float> rBX  = stackalloc float[vectorSize];
            Span<float> rBY  = stackalloc float[vectorSize];
            Span<int>   idxA = stackalloc int[vectorSize];
            Span<int>   idxB = stackalloc int[vectorSize];

            // Gather body velocities into local vectors
            Span<float> vAX = stackalloc float[vectorSize];
            Span<float> vAY = stackalloc float[vectorSize];
            Span<float> wA  = stackalloc float[vectorSize];
            Span<float> vBX = stackalloc float[vectorSize];
            Span<float> vBY = stackalloc float[vectorSize];
            Span<float> wB  = stackalloc float[vectorSize];

            // Process full vector batches
            for (; batchStart + vectorSize <= contactCount; batchStart += vectorSize)
            {
                for (int k = 0; k < vectorSize; k++)
                {
                    int i = contactIndices[batchStart + k];
                    mA[k]   = vc.InvMassA[i];
                    iA[k]   = vc.InvIA[i];
                    mB[k]   = vc.InvMassB[i];
                    iB[k]   = vc.InvIB[i];
                    nX[k]   = vc.Normal[i].X;
                    nY[k]   = vc.Normal[i].Y;
                    tX[k]   = vc.TangentX[i];
                    tY[k]   = vc.TangentY[i];
                    tM[k]   = vc.PointTangentMass[i].AsSpan[0];
                    nM[k]   = vc.PointNormalMass[i].AsSpan[0];
                    bias[k] = vc.PointVelocityBias[i].AsSpan[0];
                    friction[k] = vc.Friction[i];
                    tangSpeed[k] = vc.TangentSpeed[i];
                    accN[k] = vc.PointNormalImpulse[i].AsSpan[0];
                    accT[k] = vc.PointTangentImpulse[i].AsSpan[0];
                    rAX[k]  = vc.PointRelativeVelocityA[i].AsSpan[0].X;
                    rAY[k]  = vc.PointRelativeVelocityA[i].AsSpan[0].Y;
                    rBX[k]  = vc.PointRelativeVelocityB[i].AsSpan[0].X;
                    rBY[k]  = vc.PointRelativeVelocityB[i].AsSpan[0].Y;
                    idxA[k] = vc.IndexA[i];
                    idxB[k] = vc.IndexB[i];
                }

                for (int k = 0; k < vectorSize; k++)
                {
                    int a = idxA[k];
                    int b = idxB[k];
                    vAX[k] = linearVelocities[offset + a].X;
                    vAY[k] = linearVelocities[offset + a].Y;
                    wA[k]  = angularVelocities[offset + a];
                    vBX[k] = linearVelocities[offset + b].X;
                    vBY[k] = linearVelocities[offset + b].Y;
                    wB[k]  = angularVelocities[offset + b];
                }

                Vector<float> vAVecX = new(vAX);
                Vector<float> vAVecY = new(vAY);
                Vector<float> wAVec  = new(wA);
                Vector<float> vBVecX = new(vBX);
                Vector<float> vBVecY = new(vBY);
                Vector<float> wBVec  = new(wB);

                // Load constants
                Vector<float> mAVec = new(mA);
                Vector<float> iAVec = new(iA);
                Vector<float> mBVec = new(mB);
                Vector<float> iBVec = new(iB);
                Vector<float> nXVec = new(nX);
                Vector<float> nYVec = new(nY);
                Vector<float> tXVec = new(tX);
                Vector<float> tYVec = new(tY);
                Vector<float> tMVec = new(tM);
                Vector<float> nMVec = new(nM);
                Vector<float> biasVec = new(bias);
                Vector<float> frictionVec = new(friction);
                Vector<float> tangSpeedVec = new(tangSpeed);
                Vector<float> accNVec = new(accN);
                Vector<float> accTVec = new(accT);
                Vector<float> rAXVec = new(rAX);
                Vector<float> rAYVec = new(rAY);
                Vector<float> rBXVec = new(rBX);
                Vector<float> rBYVec = new(rBY);

                Vector<float> dvX = vBVecX - vAVecX - wAVec * rAYVec + wBVec * rBYVec;
                Vector<float> dvY = vBVecY - vAVecY + wAVec * rAXVec - wBVec * rBXVec;
                Vector<float> vt = dvX * tXVec + dvY * tYVec - tangSpeedVec;
                Vector<float> lambdaT = tMVec * (-vt);

                Vector<float> maxFriction = frictionVec * accNVec;
                Vector<float> newImpulseT = Vector.Min(Vector.Max(accTVec + lambdaT, -maxFriction), maxFriction);
                lambdaT = newImpulseT - accTVec;

                // Apply tangent delta directly to the in‑register velocities
                Vector<float> PTx = tXVec * lambdaT;
                Vector<float> PTy = tYVec * lambdaT;
                Vector<float> crossA = rAXVec * PTy - rAYVec * PTx;
                Vector<float> crossB = rBXVec * PTy - rBYVec * PTx;

                vAVecX -= PTx * mAVec;
                vAVecY -= PTy * mAVec;
                wAVec  -= iAVec * crossA;
                vBVecX += PTx * mBVec;
                vBVecY += PTy * mBVec;
                wBVec  += iBVec * crossB;

                dvX = vBVecX - vAVecX - wAVec * rAYVec + wBVec * rBYVec;
                dvY = vBVecY - vAVecY + wAVec * rAXVec - wBVec * rBXVec;
                Vector<float> vn = dvX * nXVec + dvY * nYVec;
                Vector<float> lambdaN = nMVec * (-vn + biasVec);
                Vector<float> newImpulseN = Vector.Max(accNVec + lambdaN, Vector<float>.Zero);
                lambdaN = newImpulseN - accNVec;

                Vector<float> PNx = nXVec * lambdaN;
                Vector<float> PNy = nYVec * lambdaN;
                // Recompute cross products with the new deltas
                crossA = rAXVec * PNy - rAYVec * PNx;
                crossB = rBXVec * PNy - rBYVec * PNx;

                // Apply normal delta to in‑register velocities
                vAVecX -= PNx * mAVec;
                vAVecY -= PNy * mAVec;
                wAVec  -= iAVec * crossA;
                vBVecX += PNx * mBVec;
                vBVecY += PNy * mBVec;
                wBVec  += iBVec * crossB;

                // Now accumulate both tangent and normal deltas into the shared arrays
                for (int k = 0; k < vectorSize; k++)
                {
                    int a = idxA[k];
                    int b = idxB[k];

                    // Tangent part
                    deltaLinAX[a] -= PTx[k] * mA[k];
                    deltaLinAY[a] -= PTy[k] * mA[k];
                    deltaAngA[a]  -= iA[k] * (rAX[k] * PTy[k] - rAY[k] * PTx[k]); // crossA tangent
                    deltaLinBX[b] += PTx[k] * mB[k];
                    deltaLinBY[b] += PTy[k] * mB[k];
                    deltaAngB[b]  += iB[k] * (rBX[k] * PTy[k] - rBY[k] * PTx[k]); // crossB tangent

                    // Normal part
                    deltaLinAX[a] -= PNx[k] * mA[k];
                    deltaLinAY[a] -= PNy[k] * mA[k];
                    deltaAngA[a]  -= iA[k] * (rAX[k] * PNy[k] - rAY[k] * PNx[k]); // crossA normal
                    deltaLinBX[b] += PNx[k] * mB[k];
                    deltaLinBY[b] += PNy[k] * mB[k];
                    deltaAngB[b]  += iB[k] * (rBX[k] * PNy[k] - rBY[k] * PNx[k]); // crossB normal

                    // Store new impulses
                    vc.PointTangentImpulse[contactIndices[batchStart + k]].AsSpan[0] = newImpulseT[k];
                    vc.PointNormalImpulse[contactIndices[batchStart + k]].AsSpan[0] = newImpulseN[k];
                }
            }
        }

        // Remainder – scalar loop
        for (int r = batchStart; r < contactCount; r++)
        {
            int i = contactIndices[r];
            int a = vc.IndexA[i];
            int b = vc.IndexB[i];

            ref var vA = ref linearVelocities[offset + a];
            ref var wA = ref angularVelocities[offset + a];
            ref var vB = ref linearVelocities[offset + b];
            ref var wB = ref angularVelocities[offset + b];

            var normal = vc.Normal[i];
            var tangent = new Vector2(vc.TangentX[i], vc.TangentY[i]);
            var friction = vc.Friction[i];
            var rA = vc.PointRelativeVelocityA[i].AsSpan[0];
            var rB = vc.PointRelativeVelocityB[i].AsSpan[0];
            var tMass = vc.PointTangentMass[i].AsSpan[0];
            var nMass = vc.PointNormalMass[i].AsSpan[0];
            var bias  = vc.PointVelocityBias[i].AsSpan[0];
            ref var accN = ref vc.PointNormalImpulse[i].AsSpan[0];
            ref var accT = ref vc.PointTangentImpulse[i].AsSpan[0];

            // Tangent
            var dv = vB + Vector2Helpers.Cross(wB, rB) - vA - Vector2Helpers.Cross(wA, rA);
            float vt = Vector2.Dot(dv, tangent) - vc.TangentSpeed[i];
            float lambdaT = tMass * (-vt);
            float maxF = friction * accN;
            float newImpulseT = Math.Clamp(accT + lambdaT, -maxF, maxF);
            lambdaT = newImpulseT - accT;
            accT = newImpulseT;

            var PT = tangent * lambdaT;
            vA -= PT * vc.InvMassA[i];
            wA -= vc.InvIA[i] * Vector2Helpers.Cross(rA, PT);
            vB += PT * vc.InvMassB[i];
            wB += vc.InvIB[i] * Vector2Helpers.Cross(rB, PT);

            // Normal – uses the *updated* velocities (just modified above)
            dv = vB + Vector2Helpers.Cross(wB, rB) - vA - Vector2Helpers.Cross(wA, rA);
            float vn = Vector2.Dot(dv, normal);
            float lambdaN = nMass * (-vn + bias);
            float newImpulseN = Math.Max(accN + lambdaN, 0);
            lambdaN = newImpulseN - accN;
            accN = newImpulseN;

            var PN = normal * lambdaN;
            vA -= PN * vc.InvMassA[i];
            wA -= vc.InvIA[i] * Vector2Helpers.Cross(rA, PN);
            vB += PN * vc.InvMassB[i];
            wB += vc.InvIB[i] * Vector2Helpers.Cross(rB, PN);
        }

        // Apply all accumulated deltas to the main velocity arrays
        for (int body = 0; body < bodyCount; body++)
        {
            linearVelocities[offset + body].X += deltaLinAX[body] + deltaLinBX[body];
            linearVelocities[offset + body].Y += deltaLinAY[body] + deltaLinBY[body];
            angularVelocities[offset + body]  += deltaAngA[body]  + deltaAngB[body];
        }
    }

    private static bool SolveSinglePointPositionConstraintsSIMD(
        in IslandData island,
        int[] contactIndices,
        int contactCount,
        ContactPositionConstraintSoA pc,
        Vector2[] positions,
        float[] angles,
        in SolverData data)
    {
        int bodyCount = island.Bodies.Count;
        if (contactCount == 0)
            return true;

        Span<Vector2> deltaPosA = stackalloc Vector2[bodyCount];
        Span<float>   deltaAngA  = stackalloc float[bodyCount];
        Span<Vector2> deltaPosB = stackalloc Vector2[bodyCount];
        Span<float>   deltaAngB  = stackalloc float[bodyCount];

        for (int i = 0; i < bodyCount; i++)
        {
            deltaPosA[i] = Vector2.Zero;
            deltaAngA[i] = 0f;
            deltaPosB[i] = Vector2.Zero;
            deltaAngB[i] = 0f;
        }

        float minSeparation = 0f;
        int vectorSize = Vector<float>.Count;

        for (int batchStart = 0; batchStart < contactCount; batchStart += vectorSize)
        {
            int batchEnd = Math.Min(batchStart + vectorSize, contactCount);
            int batchCount = batchEnd - batchStart;

            Span<int> idxA = stackalloc int[batchCount];
            Span<int> idxB = stackalloc int[batchCount];
            Span<float> mA = stackalloc float[batchCount];
            Span<float> iA = stackalloc float[batchCount];
            Span<float> mB = stackalloc float[batchCount];
            Span<float> iB = stackalloc float[batchCount];
            Span<float> nX = stackalloc float[batchCount];
            Span<float> nY = stackalloc float[batchCount];
            Span<float> pX = stackalloc float[batchCount];
            Span<float> pY = stackalloc float[batchCount];
            Span<float> sep = stackalloc float[batchCount];
            Span<float> cAX = stackalloc float[batchCount];
            Span<float> cAY = stackalloc float[batchCount];
            Span<float> cBX = stackalloc float[batchCount];
            Span<float> cBY = stackalloc float[batchCount];

            for (int k = 0; k < batchCount; k++)
            {
                int i = contactIndices[batchStart + k];
                int a = pc.IndexA[i];
                int b = pc.IndexB[i];
                idxA[k] = a;
                idxB[k] = b;
                mA[k] = pc.InvMassA[i];
                iA[k] = pc.InvIA[i];
                mB[k] = pc.InvMassB[i];
                iB[k] = pc.InvIB[i];

                // Current body states
                var cA = positions[a];
                float angA = angles[a];
                var cB = positions[b];
                float angB = angles[b];
                cAX[k] = cA.X;
                cAY[k] = cA.Y;
                cBX[k] = cB.X;
                cBY[k] = cB.Y;

                // Compute manifold point (scalar, unavoidable)
                Transform xfA = new Transform(angA);
                xfA.Position = cA - Physics.Transform.Mul(xfA.Quaternion2D, pc.LocalCenterA[i]);
                Transform xfB = new Transform(angB);
                xfB.Position = cB - Physics.Transform.Mul(xfB.Quaternion2D, pc.LocalCenterB[i]);

                PositionSolverManifoldInitialize(pc, i, 0, xfA, xfB, out var normal, out var point, out var separation);
                nX[k] = normal.X;
                nY[k] = normal.Y;
                pX[k] = point.X;
                pY[k] = point.Y;
                sep[k] = separation;
            }

            // Track minimum separation (scalar, no SIMD reduction needed)
            for (int k = 0; k < batchCount; k++)
                if (sep[k] < minSeparation)
                    minSeparation = sep[k];

            // If the batch is a full vector width, use SIMD
            if (batchCount == vectorSize)
            {
                var mAVec  = new Vector<float>(mA);
                var iAVec  = new Vector<float>(iA);
                var mBVec  = new Vector<float>(mB);
                var iBVec  = new Vector<float>(iB);
                var nXVec  = new Vector<float>(nX);
                var nYVec  = new Vector<float>(nY);
                var pXVec  = new Vector<float>(pX);
                var pYVec  = new Vector<float>(pY);
                var cAXVec = new Vector<float>(cAX);
                var cAYVec = new Vector<float>(cAY);
                var cBXVec = new Vector<float>(cBX);
                var cBYVec = new Vector<float>(cBY);

                // rA = point - centerA, rB = point - centerB
                var rAXVec = pXVec - cAXVec;
                var rAYVec = pYVec - cAYVec;
                var rBXVec = pXVec - cBXVec;
                var rBYVec = pYVec - cBYVec;

                // Baumgarte C = clamp(baumgarte * (sep + slop), -maxCorrection, 0)
                var slopVec   = new Vector<float>(PhysicsConstants.LinearSlop);
                var baumgarte = new Vector<float>(data.Baumgarte);
                var maxCorr   = new Vector<float>(-data.MaxLinearCorrection);
                var zero      = Vector<float>.Zero;
                var C = Vector.Min(zero, Vector.Max(baumgarte * (new Vector<float>(sep) + slopVec), maxCorr));

                // Effective mass: K = mA + mB + iA * rnA^2 + iB * rnB^2
                var rnA = rAXVec * nYVec - rAYVec * nXVec;
                var rnB = rBXVec * nYVec - rBYVec * nXVec;
                var K = mAVec + mBVec + iAVec * rnA * rnA + iBVec * rnB * rnB;

                // Impulse = -C / K (if K > 0, else 0)
                var useImpulse = Vector.GreaterThan(K, zero);
                var impulse = Vector.ConditionalSelect(useImpulse, -C / K, zero);

                var Px = nXVec * impulse;
                var Py = nYVec * impulse;

                // Deltas for body A and B
                var deltaCAx = -Px * mAVec;
                var deltaCAy = -Py * mAVec;
                var deltaAA  = -iAVec * (rAXVec * Py - rAYVec * Px); // cross(rA, P)
                var deltaCBx =  Px * mBVec;
                var deltaCBy =  Py * mBVec;
                var deltaAB  =  iBVec * (rBXVec * Py - rBYVec * Px); // cross(rB, P)

                // Accumulate into temporary arrays
                for (int k = 0; k < vectorSize; k++)
                {
                    int a = idxA[k];
                    int b = idxB[k];
                    deltaPosA[a] += new Vector2(deltaCAx[k], deltaCAy[k]);
                    deltaAngA[a] += deltaAA[k];
                    deltaPosB[b] += new Vector2(deltaCBx[k], deltaCBy[k]);
                    deltaAngB[b] += deltaAB[k];
                }
            }
            else
            {
                // Remainder – scalar fallback
                for (int k = 0; k < batchCount; k++)
                {
                    int a = idxA[k];
                    int b = idxB[k];
                    var normal = new Vector2(nX[k], nY[k]);
                    var point  = new Vector2(pX[k], pY[k]);
                    var rA = point - new Vector2(cAX[k], cAY[k]);
                    var rB = point - new Vector2(cBX[k], cBY[k]);

                    float C = Math.Clamp(data.Baumgarte * (sep[k] + PhysicsConstants.LinearSlop),
                                        -data.MaxLinearCorrection, 0f);
                    float rnA = Vector2Helpers.Cross(rA, normal);
                    float rnB = Vector2Helpers.Cross(rB, normal);
                    float K = mA[k] + mB[k] + iA[k] * rnA * rnA + iB[k] * rnB * rnB;
                    float impulse = K > 0f ? -C / K : 0f;
                    var P = normal * impulse;

                    deltaPosA[a] -= P * mA[k];
                    deltaAngA[a] -= iA[k] * Vector2Helpers.Cross(rA, P);
                    deltaPosB[b] += P * mB[k];
                    deltaAngB[b] += iB[k] * Vector2Helpers.Cross(rB, P);
                }
            }
        }

        // Apply all accumulated deltas to the real position/angle arrays
        for (int i = 0; i < bodyCount; i++)
        {
            positions[i] += deltaPosA[i] + deltaPosB[i];
            angles[i]    += deltaAngA[i] + deltaAngB[i];
        }

        // The position solve is considered successful if the deepest penetration is within tolerance.
        return minSeparation >= -3f * PhysicsConstants.LinearSlop;
    }

    private static bool SolveMultiPointPositionConstraints(
        in IslandData island,
        int[] contactIndices,
        int contactCount,
        ContactPositionConstraintSoA pc,
        Vector2[] positions,
        float[] angles,
        in SolverData data)
    {
        float minSeparation = 0f;

        for (int idx = 0; idx < contactCount; idx++)
        {
            int i = contactIndices[idx];
            int indexA = pc.IndexA[i];
            int indexB = pc.IndexB[i];
            float mA = pc.InvMassA[i];
            float iA = pc.InvIA[i];
            float mB = pc.InvMassB[i];
            float iB = pc.InvIB[i];
            var localCenterA = pc.LocalCenterA[i];
            var localCenterB = pc.LocalCenterB[i];
            int pointCount = pc.PointCount[i]; // will be 2

            ref var centerA = ref positions[indexA];
            ref var angleA  = ref angles[indexA];
            ref var centerB = ref positions[indexB];
            ref var angleB  = ref angles[indexB];

            for (int j = 0; j < pointCount; ++j)
            {
                Transform xfA = new Transform(angleA);
                xfA.Position = centerA - Physics.Transform.Mul(xfA.Quaternion2D, localCenterA);
                Transform xfB = new Transform(angleB);
                xfB.Position = centerB - Physics.Transform.Mul(xfB.Quaternion2D, localCenterB);

                PositionSolverManifoldInitialize(pc, i, j, xfA, xfB, out var normal, out var point, out var separation);

                minSeparation = Math.Min(minSeparation, separation);

                float C = Math.Clamp(data.Baumgarte * (separation + PhysicsConstants.LinearSlop),
                                    -data.MaxLinearCorrection, 0f);

                Vector2 rA = point - centerA;
                Vector2 rB = point - centerB;
                float rnA = Vector2Helpers.Cross(rA, normal);
                float rnB = Vector2Helpers.Cross(rB, normal);
                float K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
                float impulse = K > 0f ? -C / K : 0f;

                Vector2 P = normal * impulse;
                centerA -= P * mA;
                angleA  -= iA * Vector2Helpers.Cross(rA, P);
                centerB += P * mB;
                angleB  += iB * Vector2Helpers.Cross(rB, P);
            }
        }

        return minSeparation >= -3f * PhysicsConstants.LinearSlop;
    }

    private static void SolveAllVelocityConstraintsSIMD(
        in IslandData island,
        ContactVelocityConstraintSoA vc,
        Vector2[] linearVelocities,
        float[] angularVelocities,
        int[] singlePointContacts,
        int singlePointCount,
        int[] multiPointContacts,
        int multiPointCount)
    {
        SolveMultiPointVelocityConstraints(in island, multiPointContacts, multiPointCount, vc,
                                        linearVelocities, angularVelocities);
        SolveSinglePointVelocityConstraintsSIMD(in island, singlePointContacts, singlePointCount, vc,
                                                linearVelocities, angularVelocities);
    }

    private static bool SolveAllPositionConstraintsSIMD(
        in IslandData island,
        ContactPositionConstraintSoA pc,
        Vector2[] positions,
        float[] angles,
        in SolverData data,
        int[] singlePointContacts,
        int singlePointCount,
        int[] multiPointContacts,
        int multiPointCount)
    {
        bool ok = true;
        if (multiPointCount > 0)
            ok &= SolveMultiPointPositionConstraints(in island, multiPointContacts, multiPointCount, pc, positions, angles, in data);
        if (singlePointCount > 0)
            ok &= SolveSinglePointPositionConstraintsSIMD(in island, singlePointContacts, singlePointCount, pc, positions, angles, in data);
        return ok;
    }
}
