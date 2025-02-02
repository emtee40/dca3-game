
def variant(small_xyz, pad_xyz, small_uv):
    return f"""
                        &submitMesh<false,0, false, false, false, false, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        &submitMesh<true,0, false, false, false, false, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        &submitMesh<false,0, true, false, false, false, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        &submitMesh<true,0, true, false, false, false, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        &submitMesh<false,0, false, true, false, false, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        &submitMesh<true,0, false, true, false, false, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        &submitMesh<false,0, true, true, false, false, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        &submitMesh<true,0, true, true, false, false, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        0,
                        0,
                        0,
                        0,
                        &submitMesh<false,0, false, true, true, false, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        &submitMesh<true,0, false, true, true, false, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        &submitMesh<false,0, true, true, true, false, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        &submitMesh<true,0, true, true, true, false, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        &submitMesh<false,0, false, false, false, true, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        &submitMesh<true,0, false, false, false, true, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        &submitMesh<false,0, true, false, false, true, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        &submitMesh<true,0, true, false, false, true, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        &submitMesh<false,0, false, true, false, true, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        &submitMesh<true,0, false, true, false, true, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        &submitMesh<false,0, true, true, false, true, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        &submitMesh<true,0, true, true, false, true, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        0,
                        0,
                        0,
                        0,
                        &submitMesh<false,0, false, true, true, true, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        &submitMesh<true,0, false, true, true, true, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        &submitMesh<false,0, true, true, true, true, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        &submitMesh<true,0, true, true, true, true, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,

                        0,
                        &submitMesh<true,1, false, false, false, false, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        0,
                        &submitMesh<true,1, true, false, false, false, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        0,
                        &submitMesh<true,1, false, true, false, false, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        0,
                        &submitMesh<true,1, true, true, false, false, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        0,
                        0,
                        0,
                        0,
                        0,
                        &submitMesh<true,1, false, true, true, false, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        0,
                        &submitMesh<true,1, true, true, true, false, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        0,
                        &submitMesh<true,1, false, false, false, true, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        0,
                        &submitMesh<true,1, true, false, false, true, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        0,
                        &submitMesh<true,1, false, true, false, true, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        0,
                        &submitMesh<true,1, true, true, false, true, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        0,
                        0,
                        0,
                        0,
                        0,
                        &submitMesh<true,1, false, true, true, true, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        0,
                        &submitMesh<true,1, true, true, true, true, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,

                        0,
                        &submitMesh<true,2, false, false, false, false, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        0,
                        &submitMesh<true,2, true, false, false, false, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        0,
                        &submitMesh<true,2, false, true, false, false, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        0,
                        &submitMesh<true,2, true, true, false, false, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        0,
                        0,
                        0,
                        0,
                        0,
                        &submitMesh<true,2, false, true, true, false, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        0,
                        &submitMesh<true,2, true, true, true, false, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        0,
                        &submitMesh<true,2, false, false, false, true, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        0,
                        &submitMesh<true,2, true, false, false, true, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        0,
                        &submitMesh<true,2, false, true, false, true, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        0,
                        &submitMesh<true,2, true, true, false, true, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        0,
                        0,
                        0,
                        0,
                        0,
                        &submitMesh<true,2, false, true, true, true, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
                        0,
                        &submitMesh<true,2, true, true, true, true, {'true' if small_xyz else 'false'}, {'true' if small_xyz and pad_xyz else 'false'}, {'true' if small_uv else 'false'}>,
"""

str = ""
str += variant(False, False, False)
str += variant(False, False, True)
str += variant(False, True, False)
str += variant(False, True, True)
str += variant(True, False, False)
str += variant(True, False, True)
str += variant(True, True, False)
str += variant(True, True, True)

print(str)