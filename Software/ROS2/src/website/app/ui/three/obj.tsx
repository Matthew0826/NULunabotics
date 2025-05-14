// ObjModelAnimator.tsx
import { useEffect, useRef, useState } from 'react';
import * as THREE from 'three';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';
import { MTLLoader } from 'three/examples/jsm/loaders/MTLLoader.js';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

interface ObjModelAnimatorProps {
    objUrl: string;
    mtlUrl?: string;
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
    objUrl,
    mtlUrl,
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
    const [modelParts, setModelParts] = useState<ModelPart[]>([]);
    const [isLoading, setIsLoading] = useState<boolean>(true);
    const [error, setError] = useState<string | null>(null);

    // Scene references stored for access in animation functions
    const sceneRef = useRef<{
        scene: THREE.Scene | null;
        camera: THREE.PerspectiveCamera | null;
        renderer: THREE.WebGLRenderer | null;
        model: THREE.Group | null;
        controls: OrbitControls | null;
        animating: boolean;
    }>({
        scene: null,
        camera: null,
        renderer: null,
        model: null,
        controls: null,
        animating: false
    });

    // Function to find a specific part by name
    const getPartByName = (name: string): THREE.Object3D | null => {
        const part = modelParts.find(p => p.name === name);
        return part ? part.object : null;
    };

    // Example animation function - rotate a specific part
    const animatePart = (partName: string, axis: 'x' | 'y' | 'z', speed: number = 0.01) => {
        const part = getPartByName(partName);
        if (!part) return;

        if (axis === 'x') part.rotation.x += speed;
        if (axis === 'y') part.rotation.y += speed;
        if (axis === 'z') part.rotation.z += speed;
    };

    // Example function to move a part
    const movePart = (partName: string, axis: 'x' | 'y' | 'z', distance: number) => {
        const part = getPartByName(partName);
        if (!part) return;

        if (axis === 'x') part.position.x += distance;
        if (axis === 'y') part.position.y += distance;
        if (axis === 'z') part.position.z += distance;
    };

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
        const loadModel = () => {
            const objLoader = new OBJLoader();

            objLoader.load(
                objUrl,
                (object) => {
                    // Success callback
                    sceneRef.current.model = object;

                    // Center the model
                    const box = new THREE.Box3().setFromObject(object);
                    const center = box.getCenter(new THREE.Vector3());
                    center.x = 0;
                    center.z = 0;
                    center.y /= 2;
                    object.position.sub(center);

                    // Normalize the model size
                    const size = box.getSize(new THREE.Vector3());
                    const maxDim = Math.max(size.x, size.y, size.z);
                    object.scale.multiplyScalar(3 / maxDim);

                    // Add to scene
                    scene.add(object);

                    // Extract individual parts
                    const parts: ModelPart[] = [];
                    object.traverse((child) => {
                        if (child instanceof THREE.Mesh && child.name) {
                            parts.push({
                                name: child.name,
                                object: child
                            });
                        }
                    });

                    setModelParts(parts);
                    setIsLoading(false);

                    console.log('Model loaded successfully');
                    console.log('Available parts:', parts.map(p => p.name));
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

        // Load materials first if MTL URL is provided
        if (mtlUrl) {
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
                            sceneRef.current.model = object;

                            const box = new THREE.Box3().setFromObject(object);
                            const center = box.getCenter(new THREE.Vector3());
                            center.x = 0;
                            center.z = 0;
                            center.y /= 2;
                            object.position.sub(center);

                            const size = box.getSize(new THREE.Vector3());
                            const maxDim = Math.max(size.x, size.y, size.z);
                            object.scale.multiplyScalar(2.5 / maxDim);

                            scene.add(object);

                            const parts: ModelPart[] = [];
                            object.traverse((child) => {
                                if (child instanceof THREE.Mesh && child.name) {
                                    parts.push({
                                        name: child.name,
                                        object: child
                                    });
                                }
                            });

                            setModelParts(parts);
                            setIsLoading(false);
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
                undefined,
                (error) => {
                    console.error('Error loading MTL:', error);
                    // Fall back to loading without materials
                    loadModel();
                }
            );
        } else {
            // Load model without materials
            loadModel();
        }

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

            // Example animation - you can customize this
            // Replace with your own animation logic
            if (sceneRef.current.model) {
                // We no longer rotate the entire model here since OrbitControls handles that
                // But we can still animate specific parts
                modelParts.forEach(part => {
                    // This is where you would add your custom animations
                    // For example, if there's a part named "wheel":
                    if (part.name === "wheel") {
                        part.object.rotation.z += 0.02;
                    }
                    // Or if there's a part named "door":
                    if (part.name === "door") {
                        // Open/close animation logic here
                    }
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
    }, [objUrl, mtlUrl, backgroundColor, transparent, enableControls, controlsConfig]);

    // Example of a public method to expose animation control
    // You could add more methods as needed
    useEffect(() => {
        // This is an example of how you could expose animation methods
        // to parent components if needed
        if (typeof window !== 'undefined') {
            (window as any).animateModelPart = (partName: string, axis: 'x' | 'y' | 'z', speed: number) => {
                animatePart(partName, axis, speed);
            };

            (window as any).moveModelPart = (partName: string, axis: 'x' | 'y' | 'z', distance: number) => {
                movePart(partName, axis, distance);
            };

            // Add a method to reset the camera view
            (window as any).resetModelView = () => {
                if (sceneRef.current.controls) {
                    sceneRef.current.controls.reset();
                }
            };
        }

        return () => {
            if (typeof window !== 'undefined') {
                delete (window as any).animateModelPart;
                delete (window as any).moveModelPart;
                delete (window as any).resetModelView;
            }
        };
    }, [modelParts]);

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

export default ObjModelAnimator;