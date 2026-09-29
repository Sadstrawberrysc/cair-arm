"""Load the local Roboflow ONNX model with CUDA, without silent CPU fallback."""
import os


def load_cuda_model(model_id, api_key):
    # Roboflow's ONNX RF-DETR path needs PyCUDA for GPU-resident input tensors.
    # CPU input tensors still execute the ONNX graph on CUDA without PyCUDA.
    os.environ['ONNXRUNTIME_EXECUTION_PROVIDERS'] = 'CUDAExecutionProvider,CPUExecutionProvider'
    import onnxruntime as ort
    if 'CUDAExecutionProvider' not in ort.get_available_providers():
        raise RuntimeError('CUDAExecutionProvider unavailable in the model environment')
    from inference import get_model
    model = get_model(model_id=model_id, api_key=api_key, device='cpu')
    session = getattr(getattr(model, '_model', None), '_session', None)
    providers = session.get_providers() if session is not None else []
    if 'CUDAExecutionProvider' not in providers:
        raise RuntimeError('Roboflow model did not activate CUDAExecutionProvider: %s' % providers)
    print('Roboflow ONNX execution: %s' % ', '.join(providers), flush=True)
    return model
