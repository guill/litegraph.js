import { type Point, type ReadOnlyPoint } from "../interfaces"

interface VerletParticle {
  x: number
  y: number
  oldX: number
  oldY: number
  pinned: boolean
  mass: number
}

interface RopeConstraint {
  p1: VerletParticle
  p2: VerletParticle
  restLength: number
  stiffness: number
}

export interface RopePhysicsOptions {
  segments?: number
  gravity?: number
  damping?: number
  stiffness?: number
  iterations?: number
  mass?: number
}

export class RopePhysics {
  private particles: VerletParticle[] = []
  private constraints: RopeConstraint[] = []
  private gravity: number
  private damping: number
  private iterations: number
  private stiffness: number
  private mass: number
  private segments: number
  private totalLength: number = 0
  private anchorStart: ReadOnlyPoint = [0, 0]
  private anchorEnd: ReadOnlyPoint = [0, 0]

  constructor(options: RopePhysicsOptions = {}) {
    this.segments = options.segments ?? 10
    this.gravity = options.gravity ?? 0.5
    this.damping = options.damping ?? 0.99
    this.stiffness = options.stiffness ?? 0.8
    this.iterations = options.iterations ?? 3
    this.mass = options.mass ?? 1
  }

  initializeRope(start: ReadOnlyPoint, end: ReadOnlyPoint): void {
    this.particles = []
    this.constraints = []
    this.anchorStart = [start[0], start[1]]
    this.anchorEnd = [end[0], end[1]]

    const dx = end[0] - start[0]
    const dy = end[1] - start[1]
    const directDistance = Math.hypot(dx, dy)

    // Total rope length is 110% of direct distance
    this.totalLength = directDistance * 1.1
    const segmentLength = this.totalLength / this.segments

    // Create particles along the rope with slight sag
    for (let i = 0; i <= this.segments; i++) {
      const t = i / this.segments
      const x = start[0] + dx * t
      // Add initial sag to help form catenary
      const sagAmount = Math.sin(t * Math.PI) * directDistance * 0.05
      const y = start[1] + dy * t + sagAmount

      this.particles.push({
        x,
        y,
        oldX: x,
        oldY: y - sagAmount * 0.1, // Give slight downward velocity
        pinned: false, // Don't pin particles, we'll handle anchors separately
        mass: this.mass,
      })
    }

    // Create constraints between adjacent particles with fixed rest length
    for (let i = 0; i < this.particles.length - 1; i++) {
      const p1 = this.particles[i]
      const p2 = this.particles[i + 1]

      this.constraints.push({
        p1,
        p2,
        restLength: segmentLength,
        stiffness: 1.0, // Use full stiffness for length constraints
      })
    }
  }

  updateAnchors(start: ReadOnlyPoint, end: ReadOnlyPoint): void {
    if (this.particles.length === 0) return

    // Check if distance has changed significantly
    const dx = end[0] - start[0]
    const dy = end[1] - start[1]
    const newDistance = Math.hypot(dx, dy)
    const expectedRopeLength = newDistance * 1.1

    // Re-initialize if the distance has changed significantly
    if (Math.abs(this.totalLength - expectedRopeLength) > newDistance * 0.2) {
      this.initializeRope(start, end)
      return
    }

    // Update anchor positions
    this.anchorStart = [start[0], start[1]]
    this.anchorEnd = [end[0], end[1]]
  }

  step(deltaTime: number = 1, nodeRectangles?: Array<{ x: number, y: number, width: number, height: number }>): void {
    if (this.particles.length === 0) return

    const first = this.particles[0]
    const last = this.particles.at(-1)
    if (!last) return

    // Verlet integration
    for (const particle of this.particles) {
      const velX = (particle.x - particle.oldX) * this.damping
      const velY = (particle.y - particle.oldY) * this.damping

      particle.oldX = particle.x
      particle.oldY = particle.y

      particle.x += velX
      particle.y += velY + this.gravity * deltaTime * deltaTime * particle.mass
    }

    // Satisfy constraints multiple times for stability
    for (let iter = 0; iter < this.iterations; iter++) {
      // First constrain the endpoints to anchors
      first.x = this.anchorStart[0]
      first.y = this.anchorStart[1]
      last.x = this.anchorEnd[0]
      last.y = this.anchorEnd[1]

      // Then satisfy distance constraints
      for (const constraint of this.constraints) {
        const { p1, p2, restLength } = constraint

        const dx = p2.x - p1.x
        const dy = p2.y - p1.y
        const dist = Math.hypot(dx, dy)

        if (dist > 0) {
          // Calculate the difference from rest length
          const diff = (restLength - dist) / dist
          const offsetX = dx * diff * 0.5
          const offsetY = dy * diff * 0.5

          // Apply corrections
          p1.x -= offsetX
          p1.y -= offsetY
          p2.x += offsetX
          p2.y += offsetY
        }
      }

      // Apply collisions during constraint solving
      if (nodeRectangles) {
        for (const rect of nodeRectangles) {
          this.applyRectangleCollision(rect.x, rect.y, rect.width, rect.height)
        }
      }

      // Re-constrain endpoints after distance constraints
      first.x = this.anchorStart[0]
      first.y = this.anchorStart[1]
      last.x = this.anchorEnd[0]
      last.y = this.anchorEnd[1]
    }
  }

  getPoints(): Point[] {
    return this.particles.map(p => [p.x, p.y])
  }

  setGravity(gravity: number): void {
    this.gravity = gravity
  }

  setDamping(damping: number): void {
    this.damping = damping
  }

  setStiffness(stiffness: number): void {
    this.stiffness = stiffness
    for (const constraint of this.constraints) {
      constraint.stiffness = stiffness
    }
  }

  applyForce(particleIndex: number, forceX: number, forceY: number): void {
    if (particleIndex < 0 || particleIndex >= this.particles.length) return
    const particle = this.particles[particleIndex]
    if (particle.pinned) return

    particle.x += forceX
    particle.y += forceY
  }

  getNearestParticle(point: ReadOnlyPoint, maxDistance: number = Infinity): number | null {
    let minDist = maxDistance
    let nearestIndex: number | null = null

    for (let i = 0; i < this.particles.length; i++) {
      const particle = this.particles[i]
      const dx = point[0] - particle.x
      const dy = point[1] - particle.y
      const dist = Math.hypot(dx, dy)

      if (dist < minDist) {
        minDist = dist
        nearestIndex = i
      }
    }

    return nearestIndex
  }

  // Apply collision with a rectangle (node boundary)
  applyRectangleCollision(x: number, y: number, width: number, height: number): void {
    const margin = 3 // Small margin to prevent visual overlap
    const nodeTop = y - margin
    const nodeBottom = y + height + margin
    const nodeLeft = x - margin
    const nodeRight = x + width + margin

    for (let i = 1; i < this.particles.length - 1; i++) {
      const particle = this.particles[i]

      // Check if particle is inside the rectangle
      if (particle.x >= nodeLeft && particle.x <= nodeRight &&
        particle.y >= nodeTop && particle.y <= nodeBottom) {
        // Calculate distances to all edges
        const distTop = particle.y - nodeTop
        const distBottom = nodeBottom - particle.y
        const distLeft = particle.x - nodeLeft
        const distRight = nodeRight - particle.x

        // Find minimum distance to push particle out
        const minDist = Math.min(distTop, distBottom, distLeft, distRight)

        // Push particle to the nearest edge
        if (minDist === distTop) {
          // Push to top (most common case for ropes)
          particle.y = nodeTop
          // Kill downward velocity completely
          if (particle.oldY > particle.y) {
            particle.oldY = particle.y
          }
          // Apply friction to horizontal movement
          const velX = particle.x - particle.oldX
          particle.oldX = particle.x - velX * 0.95
        } else if (minDist === distBottom) {
          // Push to bottom
          particle.y = nodeBottom
          if (particle.oldY < particle.y) {
            particle.oldY = particle.y
          }
        } else if (minDist === distLeft) {
          // Push to left
          particle.x = nodeLeft
          if (particle.oldX > particle.x) {
            particle.oldX = particle.x
          }
        } else {
          // Push to right
          particle.x = nodeRight
          if (particle.oldX < particle.x) {
            particle.oldX = particle.x
          }
        }
      }
    }
  }
}

// Rope manager for handling multiple ropes
export class RopePhysicsManager {
  private ropes: Map<string, RopePhysics> = new Map()
  private enabled = true
  private globalOptions: RopePhysicsOptions

  constructor(options: RopePhysicsOptions = {}) {
    this.globalOptions = options
  }

  createRope(id: string, start: ReadOnlyPoint, end: ReadOnlyPoint): RopePhysics {
    const rope = new RopePhysics(this.globalOptions)
    rope.initializeRope(start, end)
    this.ropes.set(id, rope)
    return rope
  }

  getRope(id: string): RopePhysics | undefined {
    return this.ropes.get(id)
  }

  updateRope(id: string, start: ReadOnlyPoint, end: ReadOnlyPoint): void {
    const rope = this.ropes.get(id)
    if (rope) {
      rope.updateAnchors(start, end)
    } else {
      this.createRope(id, start, end)
    }
  }

  removeRope(id: string): void {
    this.ropes.delete(id)
  }

  step(deltaTime: number = 1, nodeRectangles?: Array<{ x: number, y: number, width: number, height: number }>): void {
    if (!this.enabled) return

    // Pass node rectangles to each rope's step method
    for (const rope of this.ropes.values()) {
      rope.step(deltaTime, nodeRectangles)
    }
  }

  clear(): void {
    this.ropes.clear()
  }

  setEnabled(enabled: boolean): void {
    this.enabled = enabled
  }

  isEnabled(): boolean {
    return this.enabled
  }

  setGlobalOptions(options: Partial<RopePhysicsOptions>): void {
    Object.assign(this.globalOptions, options)

    // Apply to existing ropes
    for (const rope of this.ropes.values()) {
      if (options.gravity !== undefined) rope.setGravity(options.gravity)
      if (options.damping !== undefined) rope.setDamping(options.damping)
      if (options.stiffness !== undefined) rope.setStiffness(options.stiffness)
    }
  }

  getRopeIds(): string[] {
    return Array.from(this.ropes.keys())
  }
}
