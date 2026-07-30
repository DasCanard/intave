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
import de.jpx3.intave.share.Position;
import org.junit.jupiter.api.Test;

import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class BlockIntersectionTest {
  @Test
  void visitsTheDestinationBoxForShortMovement() {
    Set<BlockPosition> blocks = visitedBetween(
      new Position(0.5, 0.0, 0.5),
      new Position(0.5, 0.5, 0.5),
      new BoundingBox(0.2, 0.5, 0.2, 0.8, 2.3, 0.8)
    );

    assertEquals(
      Set.of(new BlockPosition(0, 0, 0), new BlockPosition(0, 1, 0), new BlockPosition(0, 2, 0)),
      blocks
    );
  }

  @Test
  void includesBlocksCrossedByLongMovementWithoutDuplicates() {
    Set<BlockPosition> blocks = visitedBetween(
      new Position(0.5, 0.0, 0.5),
      new Position(0.5, 2.0, 0.5),
      new BoundingBox(0.2, 2.0, 0.2, 0.8, 3.8, 0.8)
    );

    assertEquals(
      Set.of(new BlockPosition(0, 1, 0), new BlockPosition(0, 2, 0), new BlockPosition(0, 3, 0)),
      blocks
    );
  }

  @Test
  void visitsDirectionalBoxesFromTopWhenMovingDown() {
    Set<BlockPosition> blocks = new LinkedHashSet<>();
    BlockIntersection.forEachBlockIntersectedBetweenDirectional(
      new Position(0.5, 1.0, 0.5),
      new Position(0.5, 0.7, 0.5),
      new BoundingBox(0.2, 0.7, 0.2, 0.8, 2.5, 0.8),
      (blockPosition, ignoredStep) -> {
        blocks.add(blockPosition);
        return true;
      }
    );

    assertEquals(
      List.of(
        new BlockPosition(0, 2, 0),
        new BlockPosition(0, 1, 0),
        new BlockPosition(0, 0, 0)
      ),
      List.copyOf(blocks)
    );
  }

  @Test
  void distinguishesSweptCandidatesFromPreciseIntersections() {
    Position from = new Position(0.5, 1.3, 0.5);
    Position to = new Position(0.5, 0.7, 0.5);
    BoundingBox destinationBox = new BoundingBox(0.2, 0.7, 0.2, 0.8, 2.5, 0.8);

    assertFalse(BlockIntersection.isPreciseIntersection(
      from, to, destinationBox, new BlockPosition(0, 3, 0)
    ));
    assertTrue(BlockIntersection.isPreciseIntersection(
      from, to, destinationBox, new BlockPosition(0, 1, 0)
    ));
  }

  private static Set<BlockPosition> visitedBetween(
    Position from, Position to, BoundingBox destinationBox
  ) {
    Set<BlockPosition> blocks = new LinkedHashSet<>();
    BlockIntersection.forEachBlockIntersectedBetween(
      from, to, destinationBox,
      (blockPosition, ignoredStep) -> {
        blocks.add(blockPosition);
        return true;
      }
    );
    return blocks;
  }
}
