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
import de.jpx3.intave.share.Motion;
import de.jpx3.intave.share.Position;
import de.jpx3.intave.share.RawVector3d;
import it.unimi.dsi.fastutil.longs.LongOpenHashSet;
import it.unimi.dsi.fastutil.longs.LongSet;

import static de.jpx3.intave.share.ClientMath.clamp_double;
import static de.jpx3.intave.share.ClientMath.floor;

/**
 * Finds blocks touched by an entity between two positions.
 *
 * <p>Implements the client traversal variants used by inside-block effects.</p>
 */
public final class BlockIntersection {
  private static final double SHORT_MOVEMENT_LIMIT = 0.99999F * 0.99999F;
  private static final double HIT_EPSILON = 1.0E-5F;
  private static final int MAX_TRAVEL_STEPS = 16;

  private BlockIntersection() {
  }

  public static boolean forEachBlockIntersectedBetween(
    Position from, Position to, BoundingBox destinationBox, BlockStepVisitor visitor
  ) {
    Motion movement = from.motionTo(to);
    if (movement.lengthSquared() < SHORT_MOVEMENT_LIMIT) {
      return visitBox(destinationBox, 0, null, visitor);
    }

    LongSet visited = new LongOpenHashSet();
    Position destinationMin = new Position(
      destinationBox.minX, destinationBox.minY, destinationBox.minZ
    );
    Position originMin = destinationMin.add(
      -movement.motionX, -movement.motionY, -movement.motionZ
    );
    int step = addCollisionsAlongTravel(
      visited, originMin, destinationMin, destinationBox, visitor
    );
    return step >= 0 && visitBox(destinationBox, step + 1, visited, visitor);
  }

  public static boolean forEachBlockIntersectedBetweenDirectional(
    Position from, Position to, BoundingBox destinationBox, BlockStepVisitor visitor
  ) {
    return DirectionalBlockIntersection.forEachBlockIntersectedBetween(
      from, to, destinationBox, visitor
    );
  }

  public static boolean isPreciseIntersection(
    Position from, Position to, BoundingBox destinationBox, BlockPosition blockPosition
  ) {
    return DirectionalBlockIntersection.isPreciseIntersection(
      from, to, destinationBox, blockPosition
    );
  }

  private static int addCollisionsAlongTravel(
    LongSet visited,
    Position from, Position to,
    BoundingBox destinationBox,
    BlockStepVisitor visitor
  ) {
    Motion movement = from.motionTo(to);
    int x = floor(from.getX());
    int y = floor(from.getY());
    int z = floor(from.getZ());
    int stepX = sign(movement.motionX);
    int stepY = sign(movement.motionY);
    int stepZ = sign(movement.motionZ);
    double intervalX = stepX == 0 ? Double.MAX_VALUE : stepX / movement.motionX;
    double intervalY = stepY == 0 ? Double.MAX_VALUE : stepY / movement.motionY;
    double intervalZ = stepZ == 0 ? Double.MAX_VALUE : stepZ / movement.motionZ;
    double nextX = intervalX * (stepX > 0 ? 1.0 - fraction(from.getX()) : fraction(from.getX()));
    double nextY = intervalY * (stepY > 0 ? 1.0 - fraction(from.getY()) : fraction(from.getY()));
    double nextZ = intervalZ * (stepZ > 0 ? 1.0 - fraction(from.getZ()) : fraction(from.getZ()));
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

      if (step++ > MAX_TRAVEL_STEPS) {
        break;
      }

      RawVector3d hit = clipUnitBlock(x, y, z, from, to);
      if (hit == null) {
        continue;
      }

      double hitX = clamp_double(hit.x(), x + HIT_EPSILON, x + 1.0 - HIT_EPSILON);
      double hitY = clamp_double(hit.y(), y + HIT_EPSILON, y + 1.0 - HIT_EPSILON);
      double hitZ = clamp_double(hit.z(), z + HIT_EPSILON, z + 1.0 - HIT_EPSILON);
      int endX = floor(hitX + destinationBox.maxX - destinationBox.minX);
      int endY = floor(hitY + destinationBox.maxY - destinationBox.minY);
      int endZ = floor(hitZ + destinationBox.maxZ - destinationBox.minZ);

      for (int blockX = x; blockX <= endX; blockX++) {
        for (int blockY = y; blockY <= endY; blockY++) {
          for (int blockZ = z; blockZ <= endZ; blockZ++) {
            BlockPosition blockPosition = new BlockPosition(blockX, blockY, blockZ);
            if (visited.add(blockPosition.toLong()) && !visitor.visit(blockPosition, step)) {
              return -1;
            }
          }
        }
      }
    }
    return step;
  }

  static boolean visitBox(
    BoundingBox box, int step, LongSet visited, BlockStepVisitor visitor
  ) {
    int minX = floor(box.minX);
    int minY = floor(box.minY);
    int minZ = floor(box.minZ);
    int maxX = floor(box.maxX);
    int maxY = floor(box.maxY);
    int maxZ = floor(box.maxZ);

    for (int x = minX; x <= maxX; x++) {
      for (int y = minY; y <= maxY; y++) {
        for (int z = minZ; z <= maxZ; z++) {
          BlockPosition blockPosition = new BlockPosition(x, y, z);
          if ((visited == null || !visited.contains(blockPosition.toLong()))
            && !visitor.visit(blockPosition, step)) {
            return false;
          }
        }
      }
    }
    return true;
  }

  static RawVector3d clipUnitBlock(
    int x, int y, int z, Position from, Position to
  ) {
    double movementX = to.getX() - from.getX();
    double movementY = to.getY() - from.getY();
    double movementZ = to.getZ() - from.getZ();
    double nearest = 1.0;
    nearest = clipPoint(
      nearest, movementX, movementY, movementZ,
      movementX > 1.0E-7 ? x : x + 1,
      y, y + 1, z, z + 1,
      from.getX(), from.getY(), from.getZ()
    );
    nearest = clipPoint(
      nearest, movementY, movementZ, movementX,
      movementY > 1.0E-7 ? y : y + 1,
      z, z + 1, x, x + 1,
      from.getY(), from.getZ(), from.getX()
    );
    nearest = clipPoint(
      nearest, movementZ, movementX, movementY,
      movementZ > 1.0E-7 ? z : z + 1,
      x, x + 1, y, y + 1,
      from.getZ(), from.getX(), from.getY()
    );
    if (nearest >= 1.0) {
      return null;
    }
    return new RawVector3d(
      from.getX() + nearest * movementX,
      from.getY() + nearest * movementY,
      from.getZ() + nearest * movementZ
    );
  }

  private static double clipPoint(
    double nearest,
    double movement, double otherMovementA, double otherMovementB,
    double plane,
    double minA, double maxA, double minB, double maxB,
    double start, double otherStartA, double otherStartB
  ) {
    if (Math.abs(movement) <= 1.0E-7) {
      return nearest;
    }
    double distance = (plane - start) / movement;
    double intersectionA = otherStartA + distance * otherMovementA;
    double intersectionB = otherStartB + distance * otherMovementB;
    if (0.0 < distance && distance < nearest
      && minA - 1.0E-7 < intersectionA && intersectionA < maxA + 1.0E-7
      && minB - 1.0E-7 < intersectionB && intersectionB < maxB + 1.0E-7) {
      return distance;
    }
    return nearest;
  }

  static int sign(double value) {
    return value < 0.0 ? -1 : value > 0.0 ? 1 : 0;
  }

  static double fraction(double value) {
    return value - floor(value);
  }

  @FunctionalInterface
  public interface BlockStepVisitor {
    /**
     * @return {@code false} to stop traversing
     */
    boolean visit(BlockPosition blockPosition, int step);
  }
}
