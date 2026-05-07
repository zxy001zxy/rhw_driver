from pathlib import Path
from setuptools import find_packages, setup

package_name = 'rhw_firedog_voice'
package_root = Path(__file__).parent


def collect_data_files(source_dir: str, install_dir: str):
    base = package_root / source_dir
    if not base.exists():
        return []

    groups: dict[str, list[str]] = {}
    for path in base.rglob('*'):
        if path.is_file():
            target = Path('share') / package_name / install_dir / path.relative_to(base).parent
            groups.setdefault(str(target), []).append(str(path))
    return sorted(groups.items())


data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml', 'readme.txt']),
]

data_files += collect_data_files('model', 'model')
data_files += collect_data_files('test_audio', 'test_audio')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xwqf',
    maintainer_email='xwqf@todo.todo',
    description='Offline voice command recognition and ROS 2 topic publishing based on Vosk.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'asr_intent_demonew = rhw_firedog_voice.asr_intent_demonew:main',
            'asr_intent_live = rhw_firedog_voice.asr_intent_live:main',
            'voice_control_node = rhw_firedog_voice.voice_control_node:main',
        ],
    },
)
