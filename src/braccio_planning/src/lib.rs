use pyo3::prelude::*;

/// A Python module implemented in Rust.
#[pymodule]
mod braccio_planning {
    use pyo3::prelude::*;

    /// Formats the sum of two numbers as string.
    #[pyfunction]
    fn braccio_planning(a: usize, b: usize) -> PyResult<String> {
        Ok((a + b).to_string())
    }
}
