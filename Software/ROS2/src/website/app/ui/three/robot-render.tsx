import { useEffect, useRef } from 'react';
import * as THREE from 'three';

export default function RobotRender() {

    const mountRef = useRef<HTMLDivElement>(null);
    const containerRef = useRef<HTMLDivElement>(null);

    useEffect(() => {
        if (!mountRef.current || !containerRef.current) return;

        // Initialize scene
        const scene = new THREE.Scene();

        // Set background based on props
        scene.background = null; // Transparent background

        // Get container dimensions
        const containerWidth = containerRef.current.clientWidth;
        const containerHeight = containerRef.current.clientHeight;

        // Camera setup
        const camera = new THREE.PerspectiveCamera(
            75,
            containerWidth / containerHeight,
            0.1,
            1000
        );
        camera.position.z = 5;

        // Renderer setup
        const renderer = new THREE.WebGLRenderer({
            antialias: true,
            alpha: true // Enable alpha for transparency
        });
        renderer.setSize(containerWidth, containerHeight);
        renderer.setPixelRatio(window.devicePixelRatio);
        mountRef.current.appendChild(renderer.domElement);

        // Cylinder geometry
        const geometry = new THREE.CylinderGeometry(1, 1, 3, 32);

        // Material with a slightly metallic look
        const material = new THREE.MeshStandardMaterial({
            color: 0x3498db,
            metalness: 0.5,
            roughness: 0.5,
        });

        // Create the cylinder mesh
        const cylinder = new THREE.Mesh(geometry, material);
        scene.add(cylinder);

        // Add lights
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
        scene.add(ambientLight);

        const directionalLight = new THREE.DirectionalLight(0xffffff, 1);
        directionalLight.position.set(5, 5, 5);
        scene.add(directionalLight);

        // Handle container resize
        const handleResize = () => {
            if (!containerRef.current) return;

            const width = containerRef.current.clientWidth;
            const height = containerRef.current.clientHeight;

            camera.aspect = width / height;
            camera.updateProjectionMatrix();

            renderer.setSize(width, height);
        };

        // Create ResizeObserver to watch container size changes
        const resizeObserver = new ResizeObserver(handleResize);
        resizeObserver.observe(containerRef.current);

        // Also handle window resize as a fallback
        window.addEventListener('resize', handleResize);

        // Animation loop
        let frameId: number;

        const animate = () => {
            frameId = requestAnimationFrame(animate);

            // Rotate cylinder on the x-axis
            cylinder.rotation.x += 0.01;

            renderer.render(scene, camera);
        };

        animate();

        // Clean up
        return () => {
            window.removeEventListener('resize', handleResize);
            resizeObserver.disconnect();
            cancelAnimationFrame(frameId);
            if (mountRef.current && mountRef.current.contains(renderer.domElement)) {
                mountRef.current.removeChild(renderer.domElement);
            }
        };
    }, []);

    return (
        <div ref={containerRef} style={{ width: '100%', height: '50vh', position: 'relative' }}>
            <div ref={mountRef} style={{ position: 'absolute', top: 0, left: 0, width: '100%', height: '100%' }} />
        </div>
    );
}
