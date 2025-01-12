#include <SDL2/SDL.h>
#include <box2d/box2d.h>
#include <vector>
#include <cstdlib>
#include <ctime>

const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 600;
const int NUM_ROBOTS = 200;
const int ROBOT_RADIUS = 5;
const float TIME_STEP = 1.0f / 60.0f; // 60 FPS
const int SUB_STEP_COUNT = 4;
const float SCALE = 100.0f; // 1 Box2D unit = 100 pixels

// Adjust bounds to ensure robots are within the walls
const float WALL_OFFSET = 30.0f;
const float minX = WALL_OFFSET + ROBOT_RADIUS;
const float maxX = WINDOW_WIDTH - WALL_OFFSET - ROBOT_RADIUS;
const float minY = WALL_OFFSET + ROBOT_RADIUS;
const float maxY = WINDOW_HEIGHT - WALL_OFFSET - ROBOT_RADIUS;



void SDL_RenderDrawCircle(SDL_Renderer* renderer, int x, int y, int radius) {
    for (int w = 0; w < radius * 2; w++) {
        for (int h = 0; h < radius * 2; h++) {
            int dx = radius - w; // Horizontal offset
            int dy = radius - h; // Vertical offset
            if ((dx * dx + dy * dy) <= (radius * radius)) {
                SDL_RenderDrawPoint(renderer, x + dx, y + dy);
            }
        }
    }
}



#include <cmath>
#include <vector>
#include <SDL2/SDL.h>
#include <box2d/box2d.h>

class Membrane {
public:
    struct Dot {
        b2BodyId bodyId;
    };

    struct Joint {
        b2JointId jointId;
    };

    std::vector<Dot> dots;
    std::vector<Joint> joints;

    Membrane(b2WorldId worldId, float centerX, float centerY, float radius, int numDots) {
        const float angleStep = 2 * M_PI / numDots;

        // Create membrane dots
        for (int i = 0; i < numDots; ++i) {
            float angle = i * angleStep;
            float x = centerX + radius * cos(angle);
            float y = centerY + radius * sin(angle);

            // Create dynamic body for each dot
            b2BodyDef dotBodyDef = b2DefaultBodyDef();
            dotBodyDef.type = b2_dynamicBody;
            dotBodyDef.position = {x, y};
            b2BodyId dotBodyId = b2CreateBody(worldId, &dotBodyDef);

            // Create circle fixture
            b2Circle dotShape;
            dotShape.center = {0.0f, 0.0f};
            dotShape.radius = ROBOT_RADIUS / SCALE;

            b2ShapeDef dotShapeDef = b2DefaultShapeDef();
            dotShapeDef.density = 5.0f; // Increased density
            dotShapeDef.friction = 0.3f;
            b2CreateCircleShape(dotBodyId, &dotShapeDef, &dotShape);

            dots.push_back({dotBodyId});
        }

        // Connect dots with distance joints (adjacent connections)
        for (int i = 0; i < numDots; ++i) {
            b2BodyId dotA = dots[i].bodyId;
            b2BodyId dotB = dots[(i + 1) % numDots].bodyId; // Connect in a loop

            b2DistanceJointDef jointDef = b2DefaultDistanceJointDef();
            jointDef.bodyIdA = dotA;
            jointDef.bodyIdB = dotB;
            jointDef.localAnchorA = {0.0f, 0.0f};
            jointDef.localAnchorB = {0.0f, 0.0f};
            jointDef.length = radius * 2 * M_PI / numDots;
            jointDef.hertz = 30.0f; // Increased stiffness
            jointDef.dampingRatio = 0.1f; // Reduced damping

            b2JointId jointId = b2CreateDistanceJoint(worldId, &jointDef);
            joints.push_back({jointId});
        }

        // Add limited cross-joints for more rigidity
        int crossJointRadius = 3; // Number of neighbors to skip for cross-joints
        for (int i = 0; i < numDots; ++i) {
            for (int j = 1; j <= crossJointRadius; ++j) {
                b2BodyId dotA = dots[i].bodyId;
                b2BodyId dotB = dots[(i + j) % numDots].bodyId;

                b2Vec2 posA = b2Body_GetPosition(dotA);
                b2Vec2 posB = b2Body_GetPosition(dotB);

                b2DistanceJointDef crossJointDef = b2DefaultDistanceJointDef();
                crossJointDef.bodyIdA = dotA;
                crossJointDef.bodyIdB = dotB;
                crossJointDef.localAnchorA = {0.0f, 0.0f};
                crossJointDef.localAnchorB = {0.0f, 0.0f};
                crossJointDef.length = sqrt(pow(posA.x - posB.x, 2) + pow(posA.y - posB.y, 2));
                crossJointDef.hertz = 20.0f; // Slightly lower stiffness
                crossJointDef.dampingRatio = 0.1f;

                b2JointId crossJointId = b2CreateDistanceJoint(worldId, &crossJointDef);
                joints.push_back({crossJointId});
            }
        }
    }

    void render(SDL_Renderer* renderer, b2WorldId worldId) {
        // Draw dots
        SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255); // Green for dots
        for (const auto& dot : dots) {
            b2Vec2 position = b2Body_GetPosition(dot.bodyId);
            SDL_RenderDrawCircle(renderer, position.x * SCALE, position.y * SCALE, ROBOT_RADIUS);
        }

        // Draw joints
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255); // White for joints
        for (const auto& joint : joints) {
            b2Vec2 posA = b2Body_GetPosition(dots[0].bodyId); // Get the positions
            b2Vec2 posB = b2Body_GetPosition(dots[1].bodyId); 

            SDL_RenderDrawLine(renderer,
                posA.x * SCALE, posA.y * SCALE,
                posB.x * SCALE, posB.y * SCALE);
        }
    }
};



struct Robot {
    b2BodyId bodyId;
    b2ShapeId shapeId;

    Robot(b2WorldId worldId, float startX, float startY) {
        // Create the body
        b2BodyDef bodyDef = b2DefaultBodyDef();
        bodyDef.type = b2_dynamicBody;
        bodyDef.position = {startX / SCALE, startY / SCALE};
        bodyDef.linearDamping = 0.0f;
        bodyDef.isBullet = true; // Enable bullet mode
        bodyId = b2CreateBody(worldId, &bodyDef);

        // Create the circle shape
        b2Circle circle;
        circle.center = {0.0f, 0.0f};
        circle.radius = ROBOT_RADIUS / SCALE; // Scaled radius

        b2ShapeDef shapeDef = b2DefaultShapeDef();
        shapeDef.density = 1.0f;
        shapeDef.friction = 0.3f;
        shapeDef.restitution = 0.8f; // Bounciness
        shapeDef.enablePreSolveEvents = true; // Enable CCD
        shapeId = b2CreateCircleShape(bodyId, &shapeDef, &circle);

        // Assign initial velocity
        b2Vec2 velocity = {(std::rand() % 200 - 100) / 20.0f, (std::rand() % 200 - 100) / 20.0f};
        b2Body_SetLinearVelocity(bodyId, velocity);
    }

    void render(SDL_Renderer* renderer, b2WorldId worldId) const {
        b2Vec2 position = b2Body_GetPosition(bodyId);

        // Convert to screen coordinates
        float screenX = position.x * SCALE;
        float screenY = position.y * SCALE;

        // Draw circle at the robot's position
        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
        for (int w = 0; w < 2 * ROBOT_RADIUS; w++) {
            for (int h = 0; h < 2 * ROBOT_RADIUS; h++) {
                int dx = ROBOT_RADIUS - w;
                int dy = ROBOT_RADIUS - h;
                if ((dx * dx + dy * dy) <= (ROBOT_RADIUS * ROBOT_RADIUS)) {
                    SDL_RenderDrawPoint(renderer, screenX + dx, screenY + dy);
                }
            }
        }
    }
};


void createWalls(b2WorldId worldId) {
    const float WALL_THICKNESS = 30.0f / SCALE; // Thickness of the wall in Box2D units (30 pixels)
    const float offset = 30.0f / SCALE;        // Offset from the window edge in Box2D units
    const float width = (WINDOW_WIDTH - 2 * 30) / SCALE; // Width adjusted for 30-pixel offset
    const float height = (WINDOW_HEIGHT - 2 * 30) / SCALE; // Height adjusted for 30-pixel offset

    // Define the static body for each wall
    b2BodyDef wallBodyDef = b2DefaultBodyDef();
    wallBodyDef.type = b2_staticBody;

    // Bottom wall
    wallBodyDef.position = {offset + width / 2, offset - WALL_THICKNESS / 2};
    b2BodyId bottomWall = b2CreateBody(worldId, &wallBodyDef);

    b2Polygon bottomShape = b2MakeBox(width / 2, WALL_THICKNESS / 2);
    b2ShapeDef bottomShapeDef = b2DefaultShapeDef();
    bottomShapeDef.friction = 0.3f;
    b2CreatePolygonShape(bottomWall, &bottomShapeDef, &bottomShape);

    // Top wall
    wallBodyDef.position = {offset + width / 2, offset + height + WALL_THICKNESS / 2};
    b2BodyId topWall = b2CreateBody(worldId, &wallBodyDef);

    b2Polygon topShape = b2MakeBox(width / 2, WALL_THICKNESS / 2);
    b2ShapeDef topShapeDef = b2DefaultShapeDef();
    topShapeDef.friction = 0.3f;
    b2CreatePolygonShape(topWall, &topShapeDef, &topShape);

    // Left wall
    wallBodyDef.position = {offset - WALL_THICKNESS / 2, offset + height / 2};
    b2BodyId leftWall = b2CreateBody(worldId, &wallBodyDef);

    b2Polygon leftShape = b2MakeBox(WALL_THICKNESS / 2, height / 2);
    b2ShapeDef leftShapeDef = b2DefaultShapeDef();
    leftShapeDef.friction = 0.3f;
    b2CreatePolygonShape(leftWall, &leftShapeDef, &leftShape);

    // Right wall
    wallBodyDef.position = {offset + width + WALL_THICKNESS / 2, offset + height / 2};
    b2BodyId rightWall = b2CreateBody(worldId, &wallBodyDef);

    b2Polygon rightShape = b2MakeBox(WALL_THICKNESS / 2, height / 2);
    b2ShapeDef rightShapeDef = b2DefaultShapeDef();
    rightShapeDef.friction = 0.3f;
    b2CreatePolygonShape(rightWall, &rightShapeDef, &rightShape);
}



void renderWalls(SDL_Renderer* renderer) {
    // Define wall positions in screen coordinates
    const int WALL_OFFSET = 30;
    SDL_Rect topWall = {WALL_OFFSET, WALL_OFFSET, WINDOW_WIDTH - 2 * WALL_OFFSET, 2}; // Top
    SDL_Rect bottomWall = {WALL_OFFSET, WINDOW_HEIGHT - WALL_OFFSET - 2, WINDOW_WIDTH - 2 * WALL_OFFSET, 2}; // Bottom
    SDL_Rect leftWall = {WALL_OFFSET, WALL_OFFSET, 2, WINDOW_HEIGHT - 2 * WALL_OFFSET}; // Left
    SDL_Rect rightWall = {WINDOW_WIDTH - WALL_OFFSET - 2, WALL_OFFSET, 2, WINDOW_HEIGHT - 2 * WALL_OFFSET}; // Right

    // Draw walls
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255); // White color for walls
    SDL_RenderFillRect(renderer, &topWall);
    SDL_RenderFillRect(renderer, &bottomWall);
    SDL_RenderFillRect(renderer, &leftWall);
    SDL_RenderFillRect(renderer, &rightWall);
}

int main(int argc, char* argv[]) {
    // Initialize SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        SDL_Log("Failed to initialize SDL: %s", SDL_GetError());
        return -1;
    }

    SDL_Window* window = SDL_CreateWindow("Swarm Robotics Simulator with Walls",
                                          SDL_WINDOWPOS_CENTERED,
                                          SDL_WINDOWPOS_CENTERED,
                                          WINDOW_WIDTH, WINDOW_HEIGHT,
                                          SDL_WINDOW_SHOWN);
    if (!window) {
        SDL_Log("Failed to create window: %s", SDL_GetError());
        SDL_Quit();
        return -1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        SDL_Log("Failed to create renderer: %s", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return -1;
    }

    // Initialize Box2D world
    b2Vec2 gravity = {0.0f, 0.0f}; // No gravity for robots
    b2WorldDef worldDef = b2DefaultWorldDef();
    worldDef.gravity = gravity;
    b2WorldId worldId = b2CreateWorld(&worldDef);

    // Create arena walls
    createWalls(worldId);

    float membraneRadius = 100.0f / SCALE; // Membrane radius in Box2D units
    int numDots = 80; // Number of dots in the membrane
    Membrane membrane(worldId, WINDOW_WIDTH / (2 * SCALE), WINDOW_HEIGHT / (2 * SCALE), membraneRadius, numDots);

    // Create robots
    std::vector<Robot> robots;
    std::srand(std::time(nullptr));
    for (int i = 0; i < NUM_ROBOTS; ++i) {
        // Generate random positions within the valid range
        float x = minX + std::rand() % static_cast<int>(maxX - minX);
        float y = minY + std::rand() % static_cast<int>(maxY - minY);
        robots.emplace_back(worldId, x, y);
    }

    // Timer variable to track elapsed time
    Uint32 lastUpdateTime = SDL_GetTicks();

    // Main loop
    bool running = true;
    SDL_Event event;
    while (running) {
        // Event handling
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
            }
        }

        // Update robot velocities every 2 seconds
        Uint32 currentTime = SDL_GetTicks();
        if (currentTime - lastUpdateTime >= 2000) { // 2 seconds
            for (auto& robot : robots) {
                // Generate random velocity
                float randomVelX = (std::rand() % 200 - 100) / 20.0f; // Random value between -10 and 10
                float randomVelY = (std::rand() % 200 - 100) / 20.0f;

                // Set new velocity
                b2Vec2 newVelocity = {randomVelX, randomVelY};
                b2Body_SetLinearVelocity(robot.bodyId, newVelocity);
            }

            // Reset the timer
            lastUpdateTime = currentTime;
        }

        // Step the Box2D world
        b2World_Step(worldId, TIME_STEP, SUB_STEP_COUNT);

        // Render
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255); // Black background
        SDL_RenderClear(renderer);

        renderWalls(renderer); // Render the walls
        membrane.render(renderer, worldId);

        for (const auto& robot : robots) {
            robot.render(renderer, worldId);
        }

        SDL_RenderPresent(renderer);
        SDL_Delay(16); // ~60 FPS
    }

    // Cleanup
    b2DestroyWorld(worldId);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}

