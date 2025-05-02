import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor


def spin_nodes(*nodes, shutdown_callback=lambda n: None, is_async=False):
    """
    Spins the given nodes in a multi-threaded executor, or spins them syncronously if there's one node or is_async is false.
    Node that if multiple nodes are passed, it'll use a multi-threaded executor.
    
    shutdown_callback is a function that will be called on each node before shutting it down.
    
    The try/except block is used to catch keyboard interrupts (Ctrl+C) and shut down the nodes gracefully.
    """
    # make sure nodes is not empty
    if len(nodes) == 0:
        raise ValueError("At least one node must be provided to spin node.")
    
    executor = None
    context = nodes[0].context if nodes else rclpy.get_default_context()
    # begin spinning the node, but catch keyboard interrupts such as Ctrl+C
    try:
        if len(nodes) == 1 and not is_async:
            rclpy.spin(nodes[0])
        else:
            # use multi threaded executor so that the node can run async functions
            executor = MultiThreadedExecutor()
            # add each node to the executor
            for node in nodes:
                executor.add_node(node)
            executor.spin()
    except KeyboardInterrupt:
        # logging could be added here if needed
        # this occurs when the user presses Ctrl+C
        pass
    except Exception as e:
        for node in nodes:
            try:
                print(f"[{type(node).__name__}] Unexpected exception: {e}")
                if rclpy.ok(context=node.context):
                    node.get_logger().error(f'Unexpected exception: {e}')
            except Exception:
                print(f"[{type(node).__name__}] Logger unavailable during exception.")
        # raise  # re-raise to propagate crash
    finally:
        if executor is not None:
            executor.shutdown()
        # shut the each node down and destroy it
        for node in nodes:
            shutdown_callback(node)
            node.destroy_node()
        try:
            if rclpy.ok(context=context):
                rclpy.shutdown(context=context)
        except Exception:
            pass  # Ignore if already shut down
