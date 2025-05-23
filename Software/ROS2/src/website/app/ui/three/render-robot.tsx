"use client";

// Render3DRobotModel.tsx
import { useEffect, useMemo, useRef, useState } from 'react';
import * as THREE from 'three';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';
import { MTLLoader } from 'three/examples/jsm/loaders/MTLLoader.js';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { Group, Mesh } from "three";
import { ObstacleType } from "@/app/types/map-objects";
import { useRobotContext } from "@/app/contexts/robot-context";

interface Render3DRobotModelProps {
    baseFilename: string;
    wheelFilename: string;
    excavatorFilename: string;
    jointRFilename: string;
    jointLFilename: string;
    motorsFilename: string;
    backgroundColor?: string;
    transparent?: boolean;
    width?: string | number;
    height?: string | number;
    enableControls?: boolean;
    controlsConfig?: {
        enableZoom?: boolean;
        enablePan?: boolean;
        autoRotate?: boolean;
        autoRotateSpeed?: number;
        dampingFactor?: number;
        maxPolarAngle?: number;
        minPolarAngle?: number;
    }
}

const Render3DRobotModel = ({
    baseFilename,
    wheelFilename,
    excavatorFilename,
    jointRFilename,
    jointLFilename,
    motorsFilename,
    backgroundColor = '#ffffff',
    transparent = true,
    width = '100%',
    height = '100%',
    enableControls = true,
    controlsConfig = {
        enableZoom: true,
        enablePan: true,
        autoRotate: false,
        autoRotateSpeed: 1.0,
        dampingFactor: 0.05,
        maxPolarAngle: Math.PI,
        minPolarAngle: 0
    }
}: Render3DRobotModelProps) => {
    const mountRef = useRef<HTMLDivElement>(null);
    const containerRef = useRef<HTMLDivElement>(null);
    const [isLoading, setIsLoading] = useState<boolean>(true);
    const [error, setError] = useState<string | null>(null);
    const [cameraPosition, setCameraPosition] = useState<THREE.Vector3>(new THREE.Vector3(0, 0, 5));

    const robotStateRef = useRef<{
        leftWheelSpeed: number;
        rightWheelSpeed: number;
        excavatorPosition: number;
        rotation: number;
        robotPosition: THREE.Vector3;
    }>({
        leftWheelSpeed: 0,
        rightWheelSpeed: 0,
        excavatorPosition: 0,
        rotation: 0,
        robotPosition: new THREE.Vector3(0, 0, 0)
    });

    const MODEL_SCALE = 1;

    const {
        leftWheelSpeed,
        rightWheelSpeed,
        excavatorPosition,
        lidarPoints,
        lidarOrigin,
        obstacles,
        robot
    } = useRobotContext();

    // Scene references stored for access in animation functions
    const sceneRef = useRef<{
        scene: THREE.Scene | null;
        camera: THREE.PerspectiveCamera | null;
        renderer: THREE.WebGLRenderer | null;
        robotObjectGroup: THREE.Group | null;
        baseModel: THREE.Group | null;
        wheelModels: THREE.Group[] | null;
        excavatorModel: THREE.Group | null;
        lidarPointsMesh: THREE.Points | null;
        obstaclesMesh: THREE.Group | null;
        terrainMesh: THREE.Mesh | null;
        controls: OrbitControls | null;
        animating: boolean;
    }>({
        scene: null,
        camera: null,
        renderer: null,
        robotObjectGroup: null,
        baseModel: null,
        wheelModels: null,
        excavatorModel: null,
        terrainMesh: null,
        lidarPointsMesh: null,
        obstaclesMesh: null,
        controls: null,
        animating: false
    });

    function createTerrainMesh(width: number, height: number, resolution: number) {
        const geometry = new THREE.PlaneGeometry(width, height, resolution, resolution);
        geometry.rotateX(-Math.PI / 2); // Face upward

        const material = new THREE.MeshStandardMaterial({ color: 0xdddddd, side: THREE.DoubleSide, flatShading: true, wireframe: true });
        return new THREE.Mesh(geometry, material);
    }

    function raiseMeshAtPoints(mesh: THREE.Mesh, obstacles: ObstacleType[], radius = 0.5, height = 0.5) {
        const posAttr = mesh.geometry.attributes.position;
        const vertex = new THREE.Vector3();


        for (let i = 0; i < posAttr.count; i++) {
            vertex.fromBufferAttribute(posAttr, i);

            // Apply effect from all points
            for (const obstacle of obstacles) {
                const dist = vertex.distanceTo(new THREE.Vector3(obstacle.x, 0, obstacle.y));
                if (dist < radius) {
                    const influence = 1 - dist / radius;
                    vertex.y += height * influence * (obstacle.isHole ? -1 : 1);
                }
            }

            posAttr.setXYZ(i, vertex.x, vertex.y, vertex.z);
        }

        posAttr.needsUpdate = true;
        mesh.geometry.computeVertexNormals(); // for lighting update
    }

    // Load and set up the 3D scene
    useEffect(() => {
        if (!mountRef.current || !containerRef.current) return;

        setIsLoading(true);
        setError(null);

        // Initialize scene
        const scene = new THREE.Scene();
        sceneRef.current.scene = scene;

        scene.background = null;

        // Get container dimensions
        const containerWidth = containerRef.current.clientWidth;
        const containerHeight = containerRef.current.clientHeight;

        // Camera setup
        const camera = new THREE.PerspectiveCamera(
            45,
            containerWidth / containerHeight,
            0.1,
            1000
        );
        camera.position.z = -17.5;
        camera.position.x = 10;
        camera.position.y = 17.5;
        sceneRef.current.camera = camera;

        // Renderer setup
        const renderer = new THREE.WebGLRenderer({
            antialias: true,
            alpha: transparent
        });
        renderer.setSize(containerWidth, containerHeight);
        renderer.setPixelRatio(window.devicePixelRatio);

        // Ensure only one canvas exists
        const existingCanvas = mountRef.current.querySelector('canvas');
        if (existingCanvas) {
            mountRef.current.removeChild(existingCanvas);
        }
        mountRef.current.appendChild(renderer.domElement);


        mountRef.current.appendChild(renderer.domElement);
        sceneRef.current.renderer = renderer;

        // Set up OrbitControls
        if (enableControls) {
            const controls = new OrbitControls(camera, renderer.domElement);

            // Configure controls based on props
            controls.enableZoom = controlsConfig.enableZoom ?? true;
            controls.enablePan = controlsConfig.enablePan ?? true;
            controls.autoRotate = controlsConfig.autoRotate ?? false;
            controls.autoRotateSpeed = controlsConfig.autoRotateSpeed ?? 1.0;
            controls.enableDamping = true;
            controls.dampingFactor = controlsConfig.dampingFactor ?? 0.05;
            controls.maxPolarAngle = controlsConfig.maxPolarAngle ?? Math.PI;
            controls.minPolarAngle = controlsConfig.minPolarAngle ?? 0;

            controls.target = new THREE.Vector3(0, 1.75, 0);

            sceneRef.current.controls = controls;
        }

        // Add lights
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
        scene.add(ambientLight);

        const directionalLight = new THREE.DirectionalLight(0xffffff, 1);
        directionalLight.position.set(5, 5, 5);
        scene.add(directionalLight);

        // Add a directional light from below for better visibility
        const bottomLight = new THREE.DirectionalLight(0xffffff, 0.3);
        bottomLight.position.set(0, -5, 0);
        scene.add(bottomLight);

        // Load OBJ file
        const loadModel = (objPath: string, loadedModel: (object: Group) => void) => {
            const objUrl = `${objPath}.obj`;
            // const mtlUrl = `${objPath}.mtl`;

                const objLoader = new OBJLoader();
                // objLoader.setMaterials(materials);

                objLoader.load(
                    objUrl,
                    (object) => {
                        // Same success handling as above
                        loadedModel(object);
                    },
                    (xhr) => {
                        // console.log(`${(xhr.loaded / xhr.total) * 100}% loaded`);
                    },
                    (error) => {
                        console.error('Error loading OBJ:', error);
                        setError('Failed to load 3D model');
                        setIsLoading(false);
                    }
                );

            // const mtlLoader = new MTLLoader();
            // mtlLoader.load(
            //     mtlUrl,
            //     (materials) => {
            //         materials.preload();
            //        const objLoader = new OBJLoader();
            //                     objLoader.setMaterials(materials);
            //
            //                     objLoader.load(
            //                         objUrl,
            //                         (object) => {
            //                             // Same success handling as above
            //                             loadedModel(object);
            //                         },
            //                         (xhr) => {
            //                             // console.log(`${(xhr.loaded / xhr.total) * 100}% loaded`);
            //                         },
            //                         (error) => {
            //                             console.error('Error loading OBJ:', error);
            //                             setError('Failed to load 3D model');
            //                             setIsLoading(false);
            //                         }
            //                     );
            //                 },
            //     // Progress callback
            //     (xhr) => {
            //         // console.log(`${(xhr.loaded / xhr.total) * 100}% loaded`);
            //     },
            //     // Error callback
            //     (error) => {
            //         console.error('Error loading OBJ:', error);
            //         setError('Failed to load 3D model');
            //         setIsLoading(false);
            //     }
            // );

        };

        sceneRef.current.robotObjectGroup = new THREE.Group();


        loadModel(baseFilename, (object: Group) => {
            sceneRef.current.baseModel = object;

            object.scale.multiplyScalar(MODEL_SCALE);

            if (!sceneRef.current.robotObjectGroup) {
                scene.add(object)
            } else {
                sceneRef.current.robotObjectGroup.add(object);
            }
        });
        loadModel(jointLFilename, (object: Group) => {
            sceneRef.current.baseModel = object;

            object.scale.multiplyScalar(MODEL_SCALE);

            if (!sceneRef.current.robotObjectGroup) {
                scene.add(object)
            } else {
                sceneRef.current.robotObjectGroup.add(object);
            }
        });
        loadModel(jointRFilename, (object: Group) => {
            sceneRef.current.baseModel = object;

            object.scale.multiplyScalar(MODEL_SCALE);

            if (!sceneRef.current.robotObjectGroup) {
                scene.add(object)
            } else {
                sceneRef.current.robotObjectGroup.add(object);
            }
        });
        loadModel(motorsFilename, (object: Group) => {
            sceneRef.current.baseModel = object;

            object.scale.multiplyScalar(MODEL_SCALE);

            if (!sceneRef.current.robotObjectGroup) {
                scene.add(object)
            } else {
                sceneRef.current.robotObjectGroup.add(object);
            }
        });


        loadModel(excavatorFilename, (object: Group) => {
            sceneRef.current.excavatorModel = object;

            object.scale.multiplyScalar(MODEL_SCALE);

            if (!sceneRef.current.robotObjectGroup) {
                scene.add(object)
            } else {
                sceneRef.current.robotObjectGroup.add(object);
            }
        });


        // Origin sphere
        const lidarOriginMesh = new THREE.Mesh(
            new THREE.CylinderGeometry(0.2, 0.2, 0.05, 16, 3),
            new THREE.MeshStandardMaterial({ color: 0xff0000 })
        );
        lidarOriginMesh.position.set(
            lidarOrigin.relPos.x,
            lidarOrigin.relPos.y,
            // lidarOrigin.relPos.y + sceneRef.current.robotObjectGroup.position.y,
            lidarOrigin.relPos.z);
        lidarOriginMesh.rotation.set(lidarOrigin.pitch, lidarOrigin.yawOffset, 0);
        sceneRef.current.robotObjectGroup.add(lidarOriginMesh);

        // Clear existing points if re-rendering
        const lidarPointGroup = new THREE.Group();
        scene.add(lidarPointGroup);

        // Clear existing points if re-rendering
        const obstaclesGroup = new THREE.Group();
        scene.add(obstaclesGroup);


        const WHEEL_OFFSET_X_FRONT = -5;  // In meters from blend file
        const WHEEL_OFFSET_X_BACK = 5.4874;  // In meters from blend file
        const WHEEL_OFFSET_Z = 5.0126;  // In meters from blend file
        const WHEEL_OFFSET_Y= -1.4637 + 3; // In meters from blend file

        const wheelPositions = [
            [1, 1],
            [WHEEL_OFFSET_X_BACK/WHEEL_OFFSET_X_FRONT, 1],
            [1, -1],
            [WHEEL_OFFSET_X_BACK/WHEEL_OFFSET_X_FRONT, -1]
        ];

        loadModel(wheelFilename, (wheel: Group) => {
            sceneRef.current.wheelModels = []

            wheelPositions.forEach(([xSign, ySign], i) => {
                const wheelClone = wheel.clone(); // clone so each wheel is independent
                wheelClone.position.set(
                    xSign * WHEEL_OFFSET_X_FRONT * MODEL_SCALE,
                    WHEEL_OFFSET_Y * MODEL_SCALE,
                    ySign * WHEEL_OFFSET_Z * MODEL_SCALE
                );

                wheelClone.scale.multiplyScalar(MODEL_SCALE);

                sceneRef.current.wheelModels?.push(wheelClone);
                if (!sceneRef.current.robotObjectGroup) {
                    scene.add(wheelClone)
                } else {
                    sceneRef.current.robotObjectGroup.add(wheelClone);

                    scene.add(sceneRef.current.robotObjectGroup);
                }

                sceneRef.current.terrainMesh?.position.set(-robot.x / 100, 0, -robot.y / 100);
            });

            setIsLoading(false);
        });

        sceneRef.current.robotObjectGroup.position.set(0, 0, 0);
        // sceneRef.current.robotObjectGroup.rotation.set(0, Math.PI/2, 0);

        const terrain = createTerrainMesh(58.7, 46.7, 50);
        scene.add(terrain);
        sceneRef.current.terrainMesh = terrain;

        // Add wireframe lines
        // const wireframe = new THREE.LineSegments(
        //     new THREE.WireframeGeometry(terrain.geometry),
        //     new THREE.LineBasicMaterial({ color: 0x000000 })
        // );
        // scene.add(wireframe); // Attach to terrain so it moves/deforms with it


        // Handle container resize
        const handleResize = () => {
            if (!containerRef.current || !sceneRef.current.camera || !sceneRef.current.renderer) return;

            const width = containerRef.current.clientWidth;
            const height = containerRef.current.clientHeight;

            sceneRef.current.camera.aspect = width / height;
            sceneRef.current.camera.updateProjectionMatrix();

            sceneRef.current.renderer.setSize(width, height);
        };

        // Create ResizeObserver
        const resizeObserver = new ResizeObserver(handleResize);
        resizeObserver.observe(containerRef.current);

        // Also handle window resize as a fallback
        window.addEventListener('resize', handleResize);

        // Animation loop
        let frameId: number;
        sceneRef.current.animating = true;

        const animate = () => {
            if (!sceneRef.current.animating) return;

            frameId = requestAnimationFrame(animate);

            // Update controls if enabled
            if (sceneRef.current.controls) {
                sceneRef.current.controls.update();
            }

            if (sceneRef.current.baseModel) {
                // We no longer rotate the entire model here since OrbitControls handles that
                // first two are front, last two are back

                const leftWheels = sceneRef.current.wheelModels?.filter(wheel => wheel.position.x < 0);
                const rightWheels = sceneRef.current.wheelModels?.filter(wheel => wheel.position.x > 0);
                leftWheels?.forEach(part => {
                    part.rotation.x -= 0.065 * robotStateRef.current.leftWheelSpeed;
                });
                rightWheels?.forEach(part => {
                    part.rotation.x -= 0.065 * robotStateRef.current.rightWheelSpeed;
                });

                // Rotate the excavator model
                if (sceneRef.current.excavatorModel && robotStateRef.current.excavatorPosition) {
                    sceneRef.current.excavatorModel.rotation.x = robotStateRef.current.excavatorPosition * -30 / 180 * Math.PI; // About 30 degrees moves it from fully up to down.
                }

                // Rotate if there's unaccounted rotation
            }
            if (sceneRef.current.robotObjectGroup) {
                if (sceneRef.current.robotObjectGroup.rotation.y !== robotStateRef.current.rotation) {
                    // Rotate the whole robot object group
                    sceneRef.current.robotObjectGroup.rotation.y = (robotStateRef.current.rotation) / 180 * Math.PI;
                }
            }

            if (sceneRef.current.terrainMesh) {
                // mOve the terrain mesh to the robot's position
                sceneRef.current.terrainMesh.position.set(-robotStateRef.current.robotPosition.x / 100, 0, -robotStateRef.current.robotPosition.y / 100);
            }

            if (sceneRef.current.renderer && sceneRef.current.scene && sceneRef.current.camera) {
                sceneRef.current.renderer.render(sceneRef.current.scene, sceneRef.current.camera);
            }
        };

        animate();

        // Clean up
        return () => {
            sceneRef.current.animating = false;
            window.removeEventListener('resize', handleResize);
            resizeObserver.disconnect();
            cancelAnimationFrame(frameId);

            if (sceneRef.current.controls) {
                sceneRef.current.controls.dispose();
            }

            if (mountRef.current && sceneRef.current.renderer) {
                if (mountRef.current.contains(sceneRef.current.renderer.domElement)) {
                    mountRef.current.removeChild(sceneRef.current.renderer.domElement);
                }
            }

        };
    }, [baseFilename, wheelFilename, excavatorFilename, backgroundColor, transparent, enableControls, controlsConfig]);

    useEffect(() => {
        robotStateRef.current = {
            leftWheelSpeed,
            rightWheelSpeed,
            excavatorPosition,
            rotation: robot.rotation,
            robotPosition: new THREE.Vector3(robot.x / 100, 0, robot.y / 100)
        }
    }, [leftWheelSpeed, rightWheelSpeed, excavatorPosition]);


    useEffect(() => {
        if (!sceneRef.current.scene) return;

        // Remove previous points mesh
        if (sceneRef.current.lidarPointsMesh) {
            sceneRef.current.scene.remove(sceneRef.current.lidarPointsMesh);
        }

        const positions: number[] = [];
        const colors: number[] = [];

        const color = new THREE.Color();

        lidarPoints.forEach(groupOfPoints => {
            groupOfPoints.forEach((point, index) => {
                const worldPoint = point.globPos.clone()
                    .sub(new THREE.Vector3(robot.x, 0, robot.y))
                    .multiplyScalar(MODEL_SCALE);
                positions.push(worldPoint.x, worldPoint.y, worldPoint.z);

                color.setHSL(index / groupOfPoints.length, 1.0, 0.5);
                colors.push(color.r, color.g, color.b);
            });
        });

        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
        geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));

        const material = new THREE.PointsMaterial({
            size: 0.12,
            transparent: true,
            opacity: 0.8,
            vertexColors: true,
        });

        const points = new THREE.Points(geometry, material);
        sceneRef.current.scene.add(points);
        sceneRef.current.lidarPointsMesh = points;
    }, [lidarPoints, lidarOrigin]);


    useEffect(() => {
        if (!sceneRef.current.scene) return;

        // Remove previous group
        if (sceneRef.current.obstaclesMesh) {
            sceneRef.current.scene.remove(sceneRef.current.obstaclesMesh);
        }

        const group = new THREE.Group();

        obstacles.forEach((obstacle: ObstacleType) => {
            const color = obstacle.isHole ? 0xcccccc : 0xff0000;

            const worldPoint = new THREE.Vector3(obstacle.x / 10, 0, obstacle.y / 10);
            worldPoint.sub(new THREE.Vector3(robot.x / 10, 0.0, robot.y / 10));

            const pointMesh = new THREE.Mesh(
                new THREE.SphereGeometry(obstacle.radius / 2.5 / 10, 8, 8),
                new THREE.MeshStandardMaterial({ color: color, transparent: true, opacity: 0.15 })
            );
            pointMesh.position.copy(worldPoint);
            group.add(pointMesh);
        });

        sceneRef.current.scene.add(group);
        sceneRef.current.obstaclesMesh = group;

        // if (sceneRef.current.mapOutlineMesh) {
        //     // ISSUES HERE
        //     raiseMeshAtPoints(sceneRef.current.mapOutlineMesh, obstacles, 0.5, 0.5);
        // }
    }, [lidarPoints, lidarOrigin]);


    return (
        <div ref={containerRef} style={{ width, height, position: 'relative' }}>
            <div ref={mountRef} style={{ position: 'absolute', top: 0, left: 0, width: '100%', height: '100%' }} />
            {isLoading && (
                <div style={{
                    position: 'absolute',
                    top: '50%',
                    left: '50%',
                    transform: 'translate(-50%, -50%)',
                    color: '#333'
                }}>
                    Loading model...
                </div>
            )}
            {error && (
                <div style={{
                    position: 'absolute',
                    top: '50%',
                    left: '50%',
                    transform: 'translate(-50%, -50%)',
                    color: 'red'
                }}>
                    {error}
                </div>
            )}
        </div>
    );
};

export default Render3DRobotModel;
