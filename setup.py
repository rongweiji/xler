from setuptools import setup, find_packages

setup(
    name="xler",
    version="0.1.0",
    description="xlerobot on device - Feetech servo motor control",
    packages=find_packages(),
    install_requires=[
        "pyserial>=3.5",
        "deepdiff>=6.0.0",
        "tqdm>=4.65.0",
    ],
    python_requires=">=3.9",
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
    ],
)
