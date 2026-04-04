/**
 * Scene presets: predefined world geometries.
 * All coordinates in NED (z increases downward, floor at z=0).
 */

import { v3Create } from '@lib/math';
import type { SceneConfig } from '@sim/core/types';

/** Open field: flat ground, no walls. 4 UWB anchors at 10x10m square. */
export function openField(): SceneConfig {
  return {
    preset: 'open_field',
    surfaceTextureQuality: 0.8,
    obstacles: [],
    sceneBounds: { min: [-5, -5, -10], max: [15, 15, 0] },
    uwbAnchors: [
      { id: 'A0', position: v3Create(0, 0, -0.5) },
      { id: 'A1', position: v3Create(10, 0, -0.5) },
      { id: 'A2', position: v3Create(10, 10, -0.5) },
      { id: 'A3', position: v3Create(0, 10, -0.5) },
    ],
  };
}

/**
 * Indoor basic: 10x10x3m room.
 * Centered at (5, 5, -1.5) in NED.
 * Floor at z=0, ceiling at z=-3.
 * Walls as thin AABBs.
 */
export function indoorBasic(): SceneConfig {
  const w = 0.2; // wall thickness
  return {
    preset: 'indoor_basic',
    surfaceTextureQuality: 0.5,
    sceneBounds: { min: [-1, -1, -4], max: [11, 11, 0] },
    obstacles: [
      // North wall (x=10)
      { min: v3Create(10, 0, -3), max: v3Create(10 + w, 10, 0) },
      // South wall (x=0)
      { min: v3Create(-w, 0, -3), max: v3Create(0, 10, 0) },
      // East wall (y=10)
      { min: v3Create(0, 10, -3), max: v3Create(10, 10 + w, 0) },
      // West wall (y=0)
      { min: v3Create(0, -w, -3), max: v3Create(10, 0, 0) },
      // Ceiling (z=-3)
      { min: v3Create(0, 0, -3 - w), max: v3Create(10, 10, -3) },
    ],
    uwbAnchors: [
      { id: 'A0', position: v3Create(0.5, 5, -2) },   // near south wall
      { id: 'A1', position: v3Create(9.5, 5, -2) },   // near north wall
      { id: 'A2', position: v3Create(5, 0.5, -2) },   // near west wall
      { id: 'A3', position: v3Create(5, 9.5, -2) },   // near east wall
    ],
  };
}

/**
 * Warehouse: 30x20x8m space with 4 column obstacles.
 * Floor at z=0, ceiling at z=-8.
 */
export function warehouse(): SceneConfig {
  const colSize = 1.0; // 1m x 1m columns
  return {
    preset: 'warehouse',
    surfaceTextureQuality: 0.6,
    sceneBounds: { min: [-1, -1, -9], max: [31, 21, 0] },
    obstacles: [
      // 4 columns at regular positions
      { min: v3Create(8, 5, -8), max: v3Create(8 + colSize, 5 + colSize, 0) },
      { min: v3Create(8, 14, -8), max: v3Create(8 + colSize, 14 + colSize, 0) },
      { min: v3Create(20, 5, -8), max: v3Create(20 + colSize, 5 + colSize, 0) },
      { min: v3Create(20, 14, -8), max: v3Create(20 + colSize, 14 + colSize, 0) },
      // Ceiling
      { min: v3Create(0, 0, -8.2), max: v3Create(30, 20, -8) },
      // Walls
      { min: v3Create(30, 0, -8), max: v3Create(30.2, 20, 0) },
      { min: v3Create(-0.2, 0, -8), max: v3Create(0, 20, 0) },
      { min: v3Create(0, 20, -8), max: v3Create(30, 20.2, 0) },
      { min: v3Create(0, -0.2, -8), max: v3Create(30, 0, 0) },
    ],
    uwbAnchors: [
      { id: 'A0', position: v3Create(1, 1, -4) },
      { id: 'A1', position: v3Create(29, 1, -4) },
      { id: 'A2', position: v3Create(29, 19, -4) },
      { id: 'A3', position: v3Create(1, 19, -4) },
      { id: 'A4', position: v3Create(15, 1, -4) },
      { id: 'A5', position: v3Create(15, 19, -4) },
    ],
  };
}

/** Get scene config for a preset name. */
export function getScenePreset(name: string): SceneConfig {
  switch (name) {
    case 'indoor_basic': return indoorBasic();
    case 'warehouse': return warehouse();
    case 'open_field':
    default: return openField();
  }
}
