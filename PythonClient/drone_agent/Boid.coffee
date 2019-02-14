# Ported almost directly from http://processingjs.org/learning/topic/flocking
# thanks a whole lot to Craig Reynolds and Daniel Shiffman

class Boid
  location: false
  velocity: false

  constructor: (loc, processing) -&gt;
    @velocity = new Vector(Math.random()*2-1,Math.random()*2-1)
    @location = loc.copy()
    @p = processing

  # Called every frame. Calculates the acceleration using the flock method,
  # and moves the boid based on it.
  step: (neighbours) -&gt;
    acceleration = this.flock(neighbours)
    @velocity.add(acceleration).limit(MAX_SPEED) # Limit the maximum speed at which a boid can go
    @location.add(@velocity)
    this._wrapIfNeeded()

  # Implements the flocking algorthim by collecting the three components
  # and returning a weighted sum.
  flock: (neighbours) -&gt;
    separation = this.separate(neighbours).multiply(SEPARATION_WEIGHT)
    alignment = this.align(neighbours).multiply(ALIGNMENT_WEIGHT)
    cohesion = this.cohere(neighbours).multiply(COHESION_WEIGHT)
    return separation.add(alignment).add(cohesion)

  # 가속도를 계산할 때 cohesion요소를 계산하기 위해 호출한다.
  cohere: (neighbours) -&gt;
    sum = new Vector
    count = 0
    for boid in neighbours
      d = @location.distance(boid.location)
      if d &gt; 0 and d &lt; NEIGHBOUR_RADIUS
        sum.add(boid.location)
        count++

    if count &gt; 0
      return this.steer_to sum.divide(count)
    else
      return sum # 아무런 영향도 주지 않기 위해 빈 벡터를 리턴한다.

  steer_to: (target) -&gt;
    desired = Vector.subtract(target, @location) # 현재 위치에서 가려 하는 곳을 가리키는 벡터
    d = desired.magnitude()  # 현재 위치에서 목적지까지의 거리는 벡터의 크기이다.

    # 만약 거리가 0보다 크면 변경할 방향을 계산한다. (아니면 0을 리턴한다.)
    if d &gt; 0
      desired.normalize()

      # 원하는 벡터의 크기를 계산하기 위한 두 옵션(1 -- 거리에 기초하여, 2 -- 최대 스피드)
      if d &lt; 100.0
        desired.multiply(MAX_SPEED*(d/100.0)) # 이 제동은 임의적으로 정했다.
      else
        desired.multiply(MAX_SPEED)

      # Steering = Desired minus Velocity
      steer = desired.subtract(@velocity)
      steer.limit(MAX_FORCE)  # 방향 전환 정도에 제한을 둔다.
    else
      steer = new Vector(0,0)

    return steer

  # Alignment component for the frame's acceleration
  align: (neighbours) -&gt;
    mean = new Vector
    count = 0
    for boid in neighbours
      d = @location.distance(boid.location)
      if d &gt; 0 and d &lt; NEIGHBOUR_RADIUS
        mean.add(boid.velocity)
        count++

    mean.divide(count) if count &gt; 0
    mean.limit(MAX_FORCE)
    return mean

  # Separation component for the frame's acceleration
  separate: (neighbours) -&gt;
    mean = new Vector
    count = 0
    for boid in neighbours
      d = @location.distance(boid.location)
      if d &gt; 0 and d &lt; DESIRED_SEPARATION
        # Normalized, weighted by distance vector pointing away from the neighbour
        mean.add Vector.subtract(@location,boid.location).normalize().divide(d)
        count++

    mean.divide(count) if count &gt; 0
    mean

  r: 2 # "radius" of the triangle
  render: () -&gt;
    # Draw a triangle rotated in the direction of velocity
    theta = @velocity.heading() + @p.radians(90)
    @p.fill(70)
    @p.stroke(255,255,0)
    @p.pushMatrix()
    @p.translate(@location.x,@location.y)
    @p.rotate(theta)
    @p.beginShape(@p.TRIANGLES)
    @p.vertex(0, -1 * @r *2)
    @p.vertex(-1 * @r, @r * 2)
    @p.vertex(@r, @r * 2)
    @p.endShape()
    @p.popMatrix()

# flock function, passed the Processing instance by Processing itself
flock = (processing) -&gt;
  start = new Vector(processing.width/2,processing.height/2)

  # Instantiate 100 boids who start in the middle of the map, have a maxmimum
  # speed of 2, maximum force of 0.05, and give them a reference to the
  # processing instance so they can render themselves.
  boids = for i in [0..100]
    new Boid(start, 2, 0.05, processing)

  processing.draw = -&gt;
    processing.background(255)
    for boid in boids
      boid.step(boids)
      boid.render()
    true

canvas = $('&lt;canvas width="550" height="550"&gt;&lt;/canvas&gt;').appendTo($('#flockingDemo'))[0]
processingInstance = new Processing(canvas, flock)

flock: (neighbours) -&gt;
  separation = this.separate(neighbours).multiply(SEPARATION_WEIGHT)
  alignment = this.align(neighbours).multiply(ALIGNMENT_WEIGHT)
  cohesion = this.cohere(neighbours).multiply(COHESION_WEIGHT)
  return separation.add(alignment).add(cohesion)