// ObjModelAnimator.tsx
import {useEffect, useRef, useState} from 'react';
import * as THREE from 'three';
import {OBJLoader} from 'three/examples/jsm/loaders/OBJLoader.js';
import {MTLLoader} from 'three/examples/jsm/loaders/MTLLoader.js';
import {OrbitControls} from 'three/examples/jsm/controls/OrbitControls.js';
import {useGamepadManagerContext} from "@/app/ui/dashboard/gamepad-state-provider";
import {Group, Object3DEventMap} from "three";

interface ObjModelAnimatorProps {
    baseFilename: string;
    wheelFilename: string;
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

interface ModelPart {
    name: string;
    object: THREE.Object3D;
}

const ObjModelAnimator = ({
                              baseFilename,
                              wheelFilename,
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
                          }: ObjModelAnimatorProps) => {
    const mountRef = useRef<HTMLDivElement>(null);
    const containerRef = useRef<HTMLDivElement>(null);
    // const [modelParts, setModelParts] = useState<ModelPart[]>([]);
    const [isLoading, setIsLoading] = useState<boolean>(true);
    const [error, setError] = useState<string | null>(null);

    const {speed} = useGamepadManagerContext();

    // Scene references stored for access in animation functions
    const sceneRef = useRef<{
        scene: THREE.Scene | null;
        camera: THREE.PerspectiveCamera | null;
        renderer: THREE.WebGLRenderer | null;
        baseModel: THREE.Group | null;
        wheelModels: THREE.Group[] | null;
        controls: OrbitControls | null;
        animating: boolean;
    }>({
        scene: null,
        camera: null,
        renderer: null,
        baseModel: null,
        wheelModels: null,
        controls: null,
        animating: false
    });

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
        camera.position.z = 5;
        sceneRef.current.camera = camera;

        // Renderer setup
        const renderer = new THREE.WebGLRenderer({
            antialias: true,
            alpha: transparent
        });
        renderer.setSize(containerWidth, containerHeight);
        renderer.setPixelRatio(window.devicePixelRatio);
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
            const mtlUrl = `${objPath}.mtl`;

            const mtlLoader = new MTLLoader();
            mtlLoader.load(
                mtlUrl,
                (materials) => {
                    materials.preload();
                    const objLoader = new OBJLoader();
                    objLoader.setMaterials(materials);

                    objLoader.load(
                        objUrl,
                        (object) => {
                            // Same success handling as above
                            loadedModel(object);

                        },
                        (xhr) => {
                            console.log(`${(xhr.loaded / xhr.total) * 100}% loaded`);
                        },
                        (error) => {
                            console.error('Error loading OBJ:', error);
                            setError('Failed to load 3D model');
                            setIsLoading(false);
                        }
                    );
                },
                // Progress callback
                (xhr) => {
                    console.log(`${(xhr.loaded / xhr.total) * 100}% loaded`);
                },
                // Error callback
                (error) => {
                    console.error('Error loading OBJ:', error);
                    setError('Failed to load 3D model');
                    setIsLoading(false);
                }
            );
        };

        const MODEL_SCALE = 0.2;
        loadModel(baseFilename, (object: Group) => {
            sceneRef.current.baseModel = object;

            // Compute bounding box
            const box = new THREE.Box3().setFromObject(object);
            const center = box.getCenter(new THREE.Vector3());

            // Recenter object to origin
            // object.position.sub(center);

            object.scale.multiplyScalar(MODEL_SCALE);
            scene.add(object);
        });

        const WHEEL_OFFSET_X = 1.72188;  // In meters from blend file
        const WHEEL_OFFSET_Z = 2.27074;  // In meters from blend file
        // const WHEEL_OFFSET_Y = -1.52909; // In meters from blend file
        const WHEEL_OFFSET_Y = -1.1233; // In meters from blend file
        // Y and z are swapped to match the model's orientation in the scene

        // const wheelPositions = [
        //     [0, 0],
        //     [-2, 2],
        //     [-2, 0],
        //     [0, 2]
        // ];
        const wheelPositions = [
            [1,   1],
            [-1,  1],
            [1,  -1],
            [-1, -1]
        ];

        loadModel(wheelFilename, (wheel: Group) => {
            sceneRef.current.wheelModels = []

            wheelPositions.forEach(([xSign, ySign], i) => {
                const wheelClone = wheel.clone(); // clone so each wheel is independent
                wheelClone.position.set(
                    xSign * WHEEL_OFFSET_X * MODEL_SCALE,
                    WHEEL_OFFSET_Y * MODEL_SCALE,
                    ySign * WHEEL_OFFSET_Z * MODEL_SCALE
                );
                // wheelClone.position.set(0,0,0)
                // wheelClone.translateX(xSign * WHEEL_OFFSET_X * MODEL_SCALE);
                // wheelClone.translateY(WHEEL_OFFSET_Y * MODEL_SCALE);
                // wheelClone.translateZ(ySign * WHEEL_OFFSET_Z * MODEL_SCALE);
                wheelClone.scale.multiplyScalar(MODEL_SCALE);

                sceneRef.current.wheelModels?.push(wheelClone);
                scene.add(wheelClone);
            });

            setIsLoading(false);
        });

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
                sceneRef.current.wheelModels?.forEach(part => {
                    part.rotation.x += 0.02 * speed;
                });
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
    }, [baseFilename, backgroundColor, transparent, enableControls, controlsConfig]);

    return (
        <div ref={containerRef} style={{width, height, position: 'relative'}}>
            <div ref={mountRef} style={{position: 'absolute', top: 0, left: 0, width: '100%', height: '100%'}}/>
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

export default ObjModelAnimator;
