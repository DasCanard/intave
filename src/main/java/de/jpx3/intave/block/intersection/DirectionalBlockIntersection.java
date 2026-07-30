/*
 * Copyright 2026 Intave
 *
 * This software is licensed under the PolyForm Perimeter License 1.0.0.
 * You may use this software for any purpose, except for providing to
 * others any product that competes with the software.
 *
 * A copy of the license is available at:
 *   https://polyformproject.org/licenses/perimeter/1.0.0/
 */

package de.jpx3.intave.block.intersection;

import de.jpx3.intave.share.BlockPosition;
import de.jpx3.intave.share.BoundingBox;
import de.jpx3.intave.share.Direction;
import de.jpx3.intave.share.Motion;
import de.jpx3.intave.share.Position;
import de.jpx3.intave.share.RawVector3d;
import it.unimi.dsi.fastutil.longs.LongOpenHashSet;
import it.unimi.dsi.fastutil.longs.LongSet;

import java.util.List;

import static de.jpx3.intave.share.ClientMath.clamp_double;
import static de.jpx3.intave.share.ClientMath.floor;

final class DirectionalBlockIntersection {
  private static final double PRECISE_MOVEMENT_LIMIT =
    0.9999900000002526 * 0.9999900000002526;
  private static final double MIN_MOVEMENT_LIMIT = 1.0E-5F * 1.0E-5F;
  private static final double HIT_EPSILON = 1.0E-5F;
  private static final int MAX_TRAVEL_STEPS = 16;

  private DirectionalBlockIntersection() {
  }

  static boolean forEachBlockIntersectedBetween(
    Position from,
    Position to,
    BoundingBox destinationBox,
    BlockIntersection.BlockStepVisitor visitor
  ) {
    Motion movement = from.motionTo(to);
    if (movement.lengthSquared() < MIN_MOVEMENT_LIMIT) {
      return BlockIntersection.visitBox(destinationBox, 0, null, visitor);
    }

    LongSet visited = new LongOpenHashSet();
    BoundingBox originBox = destinationBox.move(
      -movement.motionX, -movement.motionY, -movement.motionZ
    );
    if (!visitBox(originBox, movement, 0, visited, visitor)) {
      return false;
    }

    int step = addCollisionsAlongTravel(
      visited, movement, destinationBox, visitor
    );
    return step >= 0
      && visitBox(destinationBox, movement, step + 1, visited, visitor);
  }

  static boolean isPreciseIntersection(
    Position from,
    Position to,
    BoundingBox destinationBox,
    BlockPosition blockPosition
  ) {
    return from.distanceSquared(to) > PRECISE_MOVEMENT_LIMIT
      || destinationBox.intersectsWith(
        blockPosition.getX(), blockPosition.getY(), blockPosition.getZ(),
        blockPosition.getX() + 1.0,
        blockPosition.getY() + 1.0,
        blockPosition.getZ() + 1.0
      );
  }

  private static int addCollisionsAlongTravel(
    LongSet visited,
    Motion movement,
    BoundingBox destinationBox,
    BlockIntersection.BlockStepVisitor visitor
  ) {
    int[] corner = furthestCorner(movement);
    double sizeX = destinationBox.maxX - destinationBox.minX;
    double sizeY = destinationBox.maxY - destinationBox.minY;
    double sizeZ = destinationBox.maxZ - destinationBox.minZ;
    Position toCorner = new Position(
      destinationBox.centerX() + sizeX * 0.5 * corner[0],
      destinationBox.centerY() + sizeY * 0.5 * corner[1],
      destinationBox.centerZ() + sizeZ * 0.5 * corner[2]
    );
    Position fromCorner = toCorner.add(
      -movement.motionX, -movement.motionY, -movement.motionZ
    );
    int x = floor(fromCorner.getX());
    int y = floor(fromCorner.getY());
    int z = floor(fromCorner.getZ());
    int stepX = BlockIntersection.sign(movement.motionX);
    int stepY = BlockIntersection.sign(movement.motionY);
    int stepZ = BlockIntersection.sign(movement.motionZ);
    double intervalX = stepX == 0 ? Double.MAX_VALUE : stepX / movement.motionX;
    double intervalY = stepY == 0 ? Double.MAX_VALUE : stepY / movement.motionY;
    double intervalZ = stepZ == 0 ? Double.MAX_VALUE : stepZ / movement.motionZ;
    double nextX = intervalX
      * (stepX > 0
      ? 1.0 - BlockIntersection.fraction(fromCorner.getX())
      : BlockIntersection.fraction(fromCorner.getX()));
    double nextY = intervalY
      * (stepY > 0
      ? 1.0 - BlockIntersection.fraction(fromCorner.getY())
      : BlockIntersection.fraction(fromCorner.getY()));
    double nextZ = intervalZ
      * (stepZ > 0
      ? 1.0 - BlockIntersection.fraction(fromCorner.getZ())
      : BlockIntersection.fraction(fromCorner.getZ()));
    int step = 0;

    while (nextX <= 1.0 || nextY <= 1.0 || nextZ <= 1.0) {
      if (nextX < nextY) {
        if (nextX < nextZ) {
          x += stepX;
          nextX += intervalX;
        } else {
          z += stepZ;
          nextZ += intervalZ;
        }
      } else if (nextY < nextZ) {
        y += stepY;
        nextY += intervalY;
      } else {
        z += stepZ;
        nextZ += intervalZ;
      }

      if (step >= MAX_TRAVEL_STEPS) {
        break;
      }

      RawVector3d hit = BlockIntersection.clipUnitBlock(
        x, y, z, fromCorner, toCorner
      );
      if (hit == null) {
        continue;
      }

      step++;
      double hitX = clamp_double(hit.x(), x + HIT_EPSILON, x + 1.0 - HIT_EPSILON);
      double hitY = clamp_double(hit.y(), y + HIT_EPSILON, y + 1.0 - HIT_EPSILON);
      double hitZ = clamp_double(hit.z(), z + HIT_EPSILON, z + 1.0 - HIT_EPSILON);
      int endX = floor(hitX - sizeX * corner[0]);
      int endY = floor(hitY - sizeY * corner[1]);
      int endZ = floor(hitZ - sizeZ * corner[2]);

      if (!visitBox(
        x, y, z, endX, endY, endZ,
        movement, step, visited, visitor
      )) {
        return -1;
      }
    }
    return step;
  }

  private static int[] furthestCorner(Motion movement) {
    double x = Math.abs(movement.motionX);
    double y = Math.abs(movement.motionY);
    double z = Math.abs(movement.motionZ);
    int signX = movement.motionX >= 0.0 ? 1 : -1;
    int signY = movement.motionY >= 0.0 ? 1 : -1;
    int signZ = movement.motionZ >= 0.0 ? 1 : -1;
    if (x <= y && x <= z) {
      return new int[]{-signX, -signZ, signY};
    }
    if (y <= z) {
      return new int[]{signZ, -signY, -signX};
    }
    return new int[]{-signY, signX, -signZ};
  }

  private static boolean visitBox(
    BoundingBox box,
    Motion movement,
    int step,
    LongSet visited,
    BlockIntersection.BlockStepVisitor visitor
  ) {
    return visitBox(
      floor(box.minX), floor(box.minY), floor(box.minZ),
      floor(box.maxX), floor(box.maxY), floor(box.maxZ),
      movement, step, visited, visitor
    );
  }

  private static boolean visitBox(
    int firstX, int firstY, int firstZ,
    int secondX, int secondY, int secondZ,
    Motion movement,
    int step,
    LongSet visited,
    BlockIntersection.BlockStepVisitor visitor
  ) {
    int minX = Math.min(firstX, secondX);
    int minY = Math.min(firstY, secondY);
    int minZ = Math.min(firstZ, secondZ);
    int maxX = Math.max(firstX, secondX);
    int maxY = Math.max(firstY, secondY);
    int maxZ = Math.max(firstZ, secondZ);
    int startX = movement.motionX >= 0.0 ? minX : maxX;
    int startY = movement.motionY >= 0.0 ? minY : maxY;
    int startZ = movement.motionZ >= 0.0 ? minZ : maxZ;
    List<Direction.Axis> axes = Direction.axisStepOrder(movement);
    Direction.Axis firstAxis = axes.get(0);
    Direction.Axis secondAxis = axes.get(1);
    Direction.Axis thirdAxis = axes.get(2);
    Motion firstDirection =
      (firstAxis.select(movement.motionX, movement.motionY, movement.motionZ) >= 0.0
        ? firstAxis.positive() : firstAxis.negative()).normalMotion();
    Motion secondDirection =
      (secondAxis.select(movement.motionX, movement.motionY, movement.motionZ) >= 0.0
        ? secondAxis.positive() : secondAxis.negative()).normalMotion();
    Motion thirdDirection =
      (thirdAxis.select(movement.motionX, movement.motionY, movement.motionZ) >= 0.0
        ? thirdAxis.positive() : thirdAxis.negative()).normalMotion();
    int firstMax = firstAxis.select(maxX - minX, maxY - minY, maxZ - minZ);
    int secondMax = secondAxis.select(maxX - minX, maxY - minY, maxZ - minZ);
    int thirdMax = thirdAxis.select(maxX - minX, maxY - minY, maxZ - minZ);

    for (int first = 0; first <= firstMax; first++) {
      for (int second = 0; second <= secondMax; second++) {
        for (int third = 0; third <= thirdMax; third++) {
          BlockPosition blockPosition = new BlockPosition(
            startX
              + (int) firstDirection.motionX * first
              + (int) secondDirection.motionX * second
              + (int) thirdDirection.motionX * third,
            startY
              + (int) firstDirection.motionY * first
              + (int) secondDirection.motionY * second
              + (int) thirdDirection.motionY * third,
            startZ
              + (int) firstDirection.motionZ * first
              + (int) secondDirection.motionZ * second
              + (int) thirdDirection.motionZ * third
          );
          if ((visited == null || visited.add(blockPosition.toLong()))
            && !visitor.visit(blockPosition, step)) {
            return false;
          }
        }
      }
    }
    return true;
  }
}
